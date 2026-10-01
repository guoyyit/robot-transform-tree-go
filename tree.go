package tftree

import (
	"fmt"
	"sync"
	"time"
)

// Options 配置变换图的时间语义。
type Options struct {
	// MaxHistoryAge 是每条动态边保留的历史时长，超过的帧按时间淘汰。
	MaxHistoryAge time.Duration
	// MaxExtrapolation 是允许向历史范围之外外推的最大时长；0 表示禁止外推。
	MaxExtrapolation time.Duration
	// MaxSamples 是每条动态边保留的最大帧数；0 表示仅按时间淘汰。
	MaxSamples int
}

// 默认历史保留时长：MaxHistoryAge 传 0 时使用。
const defaultMaxHistoryAge = 10 * time.Second

// Tree 是进程内的本地坐标变换树服务，支持并发发布与查询。
type Tree struct {
	mu    sync.RWMutex
	edges map[string]*edge // 以 edgeKey(parent,child) 为键
	// 邻接表：frames[f] 列出所有与 f 相邻的边。
	frames map[string][]*edge
	opts   Options
}

// NewTree 使用给定配置创建变换树。
func NewTree(opts Options) *Tree {
	if opts.MaxHistoryAge == 0 {
		opts.MaxHistoryAge = defaultMaxHistoryAge
	}
	return &Tree{
		edges:  make(map[string]*edge),
		frames: make(map[string][]*edge),
		opts:   opts,
	}
}

// PublishDynamic 发布一帧 parent->child 的带时间戳动态变换。
func (t *Tree) PublishDynamic(parent, child string, ts time.Time, tr Transform) error {
	if err := validatePair(parent, child); err != nil {
		return err
	}
	t.mu.Lock()
	defer t.mu.Unlock()

	key := edgeKey(parent, child)
	e := t.edges[key]
	if e == nil {
		e = newEdge(parent, child, false)
		t.edges[key] = e
		t.frames[parent] = append(t.frames[parent], e)
		t.frames[child] = append(t.frames[child], e)
	} else if e.static {
		// 静态边升级为动态边：原地切换模式并清空静态状态，
		// 邻接表持有的指针保持有效。
		e.static = false
		e.staticPose = Transform{}
		e.history = nil
		e.everPruned = false
	}
	e.add(TimestampedTransform{Timestamp: ts, Transform: tr})

	if t.opts.MaxHistoryAge > 0 {
		newest := e.history[len(e.history)-1].Timestamp
		e.prune(newest.Add(-t.opts.MaxHistoryAge))
	}
	e.capHistory(t.opts.MaxSamples)
	return nil
}

// PublishStatic 发布一条不随时间变化的静态变换。
func (t *Tree) PublishStatic(parent, child string, tr Transform) error {
	if err := validatePair(parent, child); err != nil {
		return err
	}
	t.mu.Lock()
	defer t.mu.Unlock()

	key := edgeKey(parent, child)
	e, ok := t.edges[key]
	if !ok {
		e = newEdge(parent, child, true)
		t.edges[key] = e
		t.frames[parent] = append(t.frames[parent], e)
		t.frames[child] = append(t.frames[child], e)
	} else {
		e.static = true
		e.history = nil
		e.everPruned = false
	}
	e.staticPose = tr
	return nil
}

// RemoveTransform 移除一条变换边（同时移除其历史）。
func (t *Tree) RemoveTransform(parent, child string) {
	t.mu.Lock()
	defer t.mu.Unlock()
	key := edgeKey(parent, child)
	e, ok := t.edges[key]
	if !ok {
		return
	}
	delete(t.edges, key)
	t.frames[parent] = removeEdge(t.frames[parent], e)
	t.frames[child] = removeEdge(t.frames[child], e)
	if len(t.frames[parent]) == 0 {
		delete(t.frames, parent)
	}
	if len(t.frames[child]) == 0 {
		delete(t.frames, child)
	}
}

func removeEdge(list []*edge, target *edge) []*edge {
	out := make([]*edge, 0, len(list))
	for _, e := range list {
		if e != target {
			out = append(out, e)
		}
	}
	return out
}

// Lookup 返回 at 时刻 source 坐标系相对 target 坐标系的位姿，
// 即把 source 中的点变换到 target 中所需的 Transform。
func (t *Tree) Lookup(target, source string, at time.Time) (Transform, error) {
	t.mu.RLock()
	defer t.mu.RUnlock()

	if target == "" || source == "" {
		return Transform{}, fmt.Errorf("%w: empty frame name", ErrFrameNotFound)
	}
	if target == source {
		if !t.frameExists(target) {
			return Transform{}, fmt.Errorf("%w: %s", ErrFrameNotFound, target)
		}
		return identityTransform(), nil
	}
	if !t.frameExists(source) {
		return Transform{}, fmt.Errorf("%w: %s", ErrFrameNotFound, source)
	}
	if !t.frameExists(target) {
		return Transform{}, fmt.Errorf("%w: %s", ErrFrameNotFound, target)
	}

	path, err := t.findPath(target, source)
	if err != nil {
		return Transform{}, err
	}

	// 沿 target -> source 的无向路径逐边累积变换。
	result := identityTransform()
	for _, pe := range path {
		pose, err := pe.edge.sample(at, t.opts.MaxExtrapolation)
		if err != nil {
			return Transform{}, fmt.Errorf("%w: edge %s->%s at %s",
				err, pe.edge.parent, pe.edge.child, at.Format(time.RFC3339Nano))
		}
		if pe.forward {
			// 沿 child->parent 前进：新边表示“当前点在其父系中的位姿”，
			// 在右侧复合以累积子系内部的偏移。
			result = result.compose(pose)
		} else {
			// 沿 parent->child 反向前进：在右侧复合该边逆变换。
			result = result.compose(pose.inverse())
		}
	}
	return result, nil
}

// directedStep 表示无向路径上的一步，forward 为 true 时沿 child->parent 方向。
type directedStep struct {
	edge    *edge
	forward bool
}

// findPath 先判断有向图是否含环；含环时查询一律终止。
// 无环时图是一片有向森林，在无向意义下用 BFS 寻找唯一简单路径。
func (t *Tree) findPath(start, goal string) ([]directedStep, error) {
	if err := t.directedCycleError(); err != nil {
		return nil, err
	}

	// 无向 BFS，记录到达每个帧所走的一步。
	type arrival struct {
		frame string
		step  directedStep
	}
	prev := map[string]arrival{}
	visited := map[string]bool{start: true}
	queue := []string{start}
	found := false
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		if cur == goal {
			found = true
			break
		}
		for _, e := range t.frames[cur] {
			next := e.child
			step := directedStep{edge: e, forward: true}
			if cur == e.child {
				next = e.parent
				step.forward = false
			}
			if visited[next] {
				continue
			}
			visited[next] = true
			prev[next] = arrival{frame: cur, step: step}
			queue = append(queue, next)
		}
	}
	if !found {
		return nil, fmt.Errorf("%w: %s and %s", ErrNoConnection, start, goal)
	}

	var path []directedStep
	for cur := goal; cur != start; cur = prev[cur].frame {
		path = append(path, prev[cur].step)
	}
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path, nil
}

// directedCycleError 对有向图做三色 DFS，存在回边（指向当前递归栈上的帧）
// 时返回 ErrCycleDetected。图含环时所有查询都应终止。
func (t *Tree) directedCycleError() error {
	const (
		white = 0
		gray  = 1
		black = 2
	)
	color := make(map[string]int, len(t.frames))

	var visit func(frame string) bool
	visit = func(frame string) bool {
		color[frame] = gray
		for _, e := range t.frames[frame] {
			if e.parent != frame {
				continue // 只沿 parent->child 的出边遍历。
			}
			switch color[e.child] {
			case gray:
				return true
			case white:
				if visit(e.child) {
					return true
				}
			}
		}
		color[frame] = black
		return false
	}

	for frame := range t.frames {
		if color[frame] == white && visit(frame) {
			return fmt.Errorf("%w: graph contains a directed cycle", ErrCycleDetected)
		}
	}
	return nil
}

// validatePair 校验父子坐标系名称。
func validatePair(parent, child string) error {
	if parent == "" || child == "" {
		return fmt.Errorf("%w: empty frame name", ErrFrameNotFound)
	}
	if parent == child {
		return fmt.Errorf("%w: self-referential transform %s->%s", ErrCycleDetected, parent, child)
	}
	return nil
}

// frameExists 判断 frame 是否出现在图中。
func (t *Tree) frameExists(frame string) bool {
	_, ok := t.frames[frame]
	return ok
}

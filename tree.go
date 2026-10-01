package transformtree

import (
	"sync"
	"time"
)

// 默认配置：零值 Config 也能提供有界的行为。
const (
	defaultMaxAge           = 10 * time.Second
	defaultMaxEntries       = 1000
	defaultMaxExtrapolation = 0
)

// Config 是变换树的全局配置。
type Config struct {
	// MaxAge 限制每条动态边保留多长时间的历史，按最新帧时间淘汰。
	// 零值使用默认值 10s。
	MaxAge time.Duration
	// MaxEntries 限制每条动态边最多保留多少帧采样，超限淘汰最旧帧。
	// 零值使用默认值 1000。
	MaxEntries int
	// MaxExtrapolation 是查询时间超出历史范围（晚于最新帧）时允许的
	// 外推时间上限；零值表示禁止外推。
	MaxExtrapolation time.Duration
}

// Tree 是本地坐标变换树服务，支持并发发布与查询。
type Tree struct {
	mu    sync.RWMutex
	cfg   historyConfig
	edges map[edgeKey]*edgeState
}

type edgeKey struct {
	parent string
	child  string
}

type edgeState struct {
	parent  string
	child   string
	static  bool
	staticT Transform
	history *edgeHistory
}

// New 创建一个空的变换树。
func New(cfg Config) *Tree {
	hc := historyConfig{
		maxAge:           cfg.MaxAge,
		maxEntries:       cfg.MaxEntries,
		maxExtrapolation: cfg.MaxExtrapolation,
	}
	if hc.maxAge <= 0 {
		hc.maxAge = defaultMaxAge
	}
	if hc.maxEntries <= 0 {
		hc.maxEntries = defaultMaxEntries
	}
	if hc.maxExtrapolation < 0 {
		hc.maxExtrapolation = defaultMaxExtrapolation
	}
	return &Tree{
		cfg:   hc,
		edges: make(map[edgeKey]*edgeState),
	}
}

// Publish 发布一帧 parent -> child 的动态变换采样。
// 采样允许乱序、延迟到达：历史按时间戳有序维护，重复时间戳被忽略。
func (t *Tree) Publish(parent, child string, tr Transform, ts time.Time) {
	key := edgeKey{parent: parent, child: child}
	t.mu.Lock()
	defer t.mu.Unlock()
	e := t.edges[key]
	if e == nil || e.static {
		e = &edgeState{
			parent:  parent,
			child:   child,
			history: newEdgeHistory(t.cfg),
		}
		t.edges[key] = e
	}
	e.history.insert(tr, ts)
}

// PublishStatic 发布一条不随时间变化的静态变换，查询时直接返回。
// 重复发布会以最新定义覆盖，且不受时间戳影响。
func (t *Tree) PublishStatic(parent, child string, tr Transform) {
	key := edgeKey{parent: parent, child: child}
	t.mu.Lock()
	defer t.mu.Unlock()
	t.edges[key] = &edgeState{
		parent:  parent,
		child:   child,
		static:  true,
		staticT: tr,
	}
}

// Remove 移除一条已发布的边（动态或静态），用于解除闭环或下线来源。
func (t *Tree) Remove(parent, child string) {
	t.mu.Lock()
	defer t.mu.Unlock()
	delete(t.edges, edgeKey{parent: parent, child: child})
}

// LookupTransform 返回 at 时刻 source 坐标系在 target 坐标系中的位姿：
// 把 source 中的点变换到 target。它会沿变换树拼接中间各边，
// 动态边在 at 时刻插值/外推，静态边直接返回。
func (t *Tree) LookupTransform(target, source string, at time.Time) (Transform, error) {
	t.mu.RLock()
	defer t.mu.RUnlock()

	adj := make(map[string][]*edgeState, len(t.edges))
	for _, e := range t.edges {
		adj[e.parent] = append(adj[e.parent], e)
		adj[e.child] = append(adj[e.child], e)
	}
	if _, ok := adj[target]; !ok {
		return Transform{}, ErrFrameNotFound
	}
	if _, ok := adj[source]; !ok {
		return Transform{}, errFrame(source)
	}

	if err := detectCycle(adj); err != nil {
		return Transform{}, err
	}

	// 无环图中以 target 为根做无向 DFS，记录到达每个节点所经的边。
	parentEdge := map[string]*edgeState{}
	stack := []string{target}
	for len(stack) > 0 {
		cur := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		for _, e := range adj[cur] {
			next := otherEnd(e, cur)
			if _, ok := parentEdge[next]; ok || next == target {
				continue
			}
			parentEdge[next] = e
			stack = append(stack, next)
		}
	}
	if _, ok := parentEdge[source]; !ok {
		return Transform{}, errFrame(source)
	}

	// 从 source 沿父边回溯到 target，逐级把 source -> ... -> target 拼起来。
	result := Identity()
	for cur := source; cur != target; {
		e := parentEdge[cur]
		up := otherEnd(e, cur)
		step, err := t.edgeTransformAt(e, at)
		if err != nil {
			return Transform{}, err
		}
		if e.child == cur && e.parent == up {
			result = step.Compose(result)
		} else {
			result = step.Inverse().Compose(result)
		}
		cur = up
	}
	return result, nil
}

// edgeTransformAt 解析单条边在 at 时刻 parent <- child 的方向位姿。
func (t *Tree) edgeTransformAt(e *edgeState, at time.Time) (Transform, error) {
	if e.static {
		return e.staticT, nil
	}
	return e.history.lookup(at)
}

func otherEnd(e *edgeState, node string) string {
	if e.parent == node {
		return e.child
	}
	return e.parent
}

// detectCycle 对无向变换图做 DFS 森林遍历：
// 任何通向“已访问且非来路”节点的边都意味着闭环（含自环与平行边）。
func detectCycle(adj map[string][]*edgeState) error {
	visited := make(map[string]bool, len(adj))
	type step struct {
		node string
		from *edgeState
	}
	for start := range adj {
		if visited[start] {
			continue
		}
		visited[start] = true
		stack := []step{{node: start}}
		for len(stack) > 0 {
			cur := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			for _, e := range adj[cur.node] {
				if e == cur.from {
					continue
				}
				next := otherEnd(e, cur.node)
				if visited[next] {
					return ErrCycle
				}
				visited[next] = true
				stack = append(stack, step{node: next, from: e})
			}
		}
	}
	return nil
}

// errFrame 统一未连通/不存在帧的错误，与 ErrFrameNotFound 配合 errors.Is。
func errFrame(name string) error {
	return &frameError{name: name}
}

type frameError struct {
	name string
}

func (e *frameError) Error() string {
	return "transformtree: frame not reachable or not found: " + e.name
}

func (e *frameError) Is(target error) bool {
	return target == ErrFrameNotFound
}

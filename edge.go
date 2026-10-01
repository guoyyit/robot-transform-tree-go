package tftree

import (
	"sort"
	"time"
)

// edgeKey 是一条有向边的唯一标识。允许同一子坐标系存在多条入边，
// 从而图中可能出现环，需要在查询时检测并终止。
func edgeKey(parent, child string) string {
	return parent + "\x00" + child
}

// edge 是父坐标系到子坐标系的一条有向边。
// 静态边只保存一个位姿；动态边保存按时间戳有序的有界历史。
type edge struct {
	parent     string
	child      string
	static     bool
	staticPose Transform

	history    []TimestampedTransform
	everPruned bool
}

// newEdge 创建一条边。
func newEdge(parent, child string, static bool) *edge {
	return &edge{
		parent: parent,
		child:  child,
		static: static,
	}
}

// add 按时间戳有序插入一帧动态变换。
// 相同时间戳已存在或数据更旧时忽略，返回是否真正写入。
func (e *edge) add(tt TimestampedTransform) bool {
	if e.static {
		e.staticPose = tt.Transform
		return true
	}
	// 只接受严格新于最新一帧的数据：旧时间戳晚到一律忽略，
	// 保证查询结果与到达顺序无关、历史始终按时间单调增长。
	if len(e.history) > 0 && !tt.Timestamp.After(e.history[len(e.history)-1].Timestamp) {
		return false
	}
	idx := sort.Search(len(e.history), func(i int) bool {
		return e.history[i].Timestamp.After(tt.Timestamp)
	})
	// 命中已有时间戳（idx 指向下一帧，其前一帧即同时间戳）：重复发布忽略，
	// 保证查询结果与到达顺序无关、不会发生状态跳变。
	if idx > 0 && e.history[idx-1].Timestamp.Equal(tt.Timestamp) {
		return false
	}
	e.history = append(e.history, TimestampedTransform{})
	copy(e.history[idx+1:], e.history[idx:])
	e.history[idx] = tt
	return true
}

// prune 淘汰早于 cutoff 的历史帧，返回被淘汰的帧数。
func (e *edge) prune(cutoff time.Time) int {
	if e.static || len(e.history) == 0 {
		return 0
	}
	idx := sort.Search(len(e.history), func(i int) bool {
		return !e.history[i].Timestamp.Before(cutoff)
	})
	if idx == 0 {
		return 0
	}
	removed := idx
	e.history = append([]TimestampedTransform(nil), e.history[idx:]...)
	e.everPruned = true
	return removed
}

// capHistory 把历史帧数限制在 maxSamples 以内；maxSamples <= 0 表示不限制。
func (e *edge) capHistory(maxSamples int) {
	if maxSamples <= 0 || e.static || len(e.history) <= maxSamples {
		return
	}
	drop := len(e.history) - maxSamples
	e.history = append([]TimestampedTransform(nil), e.history[drop:]...)
	e.everPruned = true
}

// sample 在指定时间取边的位姿：静态边直接返回；
// 动态边在相邻两帧之间插值，超范围按外推/过期策略处理。
func (e *edge) sample(at time.Time, maxExtrapolation time.Duration) (Transform, error) {
	if e.static {
		return e.staticPose, nil
	}
	if len(e.history) == 0 {
		return Transform{}, ErrEmptyHistory
	}

	oldest, newest := e.history[0], e.history[len(e.history)-1]

	switch {
	case at.Equal(oldest.Timestamp):
		return oldest.Transform, nil
	case at.Equal(newest.Timestamp):
		return newest.Transform, nil
	case at.Before(oldest.Timestamp):
		// 历史曾被淘汰：该时间点可能曾有数据但已丢失，只能判定为过期。
		if e.everPruned {
			return Transform{}, ErrExpired
		}
		if maxExtrapolation >= 0 && oldest.Timestamp.Sub(at) > maxExtrapolation {
			return Transform{}, ErrExtrapolation
		}
		return oldest.Transform, nil
	case at.After(newest.Timestamp):
		if maxExtrapolation >= 0 && at.Sub(newest.Timestamp) > maxExtrapolation {
			return Transform{}, ErrExtrapolation
		}
		return newest.Transform, nil
	}

	idx := sort.Search(len(e.history), func(i int) bool {
		return e.history[i].Timestamp.After(at)
	})
	a, b := e.history[idx-1], e.history[idx]
	span := b.Timestamp.Sub(a.Timestamp)
	frac := float64(at.Sub(a.Timestamp)) / float64(span)
	return interpolate(a, b, frac), nil
}

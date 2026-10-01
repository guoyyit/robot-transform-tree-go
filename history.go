package transformtree

import (
	"sort"
	"time"
)

// historyConfig 是单条动态变换边的历史维护参数。
type historyConfig struct {
	// maxAge>0 时，每帧新数据到达会淘汰早于 newest-maxAge 的旧帧。
	maxAge time.Duration
	// maxEntries>0 时，历史帧数超过该值后淘汰最旧帧。
	maxEntries int
	// maxExtrapolation 限制查询时间超出历史范围后允许外推的时长。
	maxExtrapolation time.Duration
}

// edgeHistory 维护同一来源（同一条父子边）按时间戳有序的有界采样历史。
// 历史范围之外的查询判定为“过期”（已淘汰）或“外推”（未来/尚未到达），
// 两者返回可区分的错误。
type edgeHistory struct {
	cfg     historyConfig
	entries []stampedTransform
}

func newEdgeHistory(cfg historyConfig) *edgeHistory {
	return &edgeHistory{cfg: cfg}
}

// insert 加入一帧采样。采样按时间戳有序插入；
// 晚到的旧时间戳不会覆盖或移动更新的数据，重复时间戳整帧忽略。
func (h *edgeHistory) insert(tr Transform, ts time.Time) {
	idx := sort.Search(len(h.entries), func(i int) bool {
		return !h.entries[i].ts.Before(ts)
	})
	if idx < len(h.entries) && h.entries[idx].ts.Equal(ts) {
		return
	}
	h.entries = append(h.entries, stampedTransform{})
	copy(h.entries[idx+1:], h.entries[idx:])
	h.entries[idx] = stampedTransform{ts: ts, tr: tr}
	// 淘汰以已知最新帧为基准，晚到的旧帧若已在窗口外会立即被丢弃。
	newest := ts
	if last := h.entries[len(h.entries)-1].ts; last.After(newest) {
		newest = last
	}
	h.evict(newest)
}

// evict 以最新到达帧 ts 为基准，按 maxAge 与 maxEntries 淘汰旧帧。
func (h *edgeHistory) evict(newest time.Time) {
	drop := 0
	if h.cfg.maxAge > 0 {
		cutoff := newest.Add(-h.cfg.maxAge)
		// 保留 cutoff 本帧；严格早于 cutoff 的帧淘汰。
		drop = sort.Search(len(h.entries), func(i int) bool {
			return !h.entries[i].ts.Before(cutoff)
		})
	}
	if h.cfg.maxEntries > 0 && len(h.entries)-drop > h.cfg.maxEntries {
		byCount := len(h.entries) - h.cfg.maxEntries
		if byCount > drop {
			drop = byCount
		}
	}
	if drop > 0 {
		h.entries = append(h.entries[:0], h.entries[drop:]...)
	}
}

// lookup 返回指定时间的变换。
func (h *edgeHistory) lookup(at time.Time) (Transform, error) {
	if len(h.entries) == 0 {
		return Transform{}, ErrNoData
	}
	oldest := h.entries[0].ts
	newest := h.entries[len(h.entries)-1].ts

	if at.Before(oldest) {
		// 早于现存最早帧：所需数据要么已淘汰（过期），要么从未发布。
		return Transform{}, ErrExpired
	}
	if !at.After(newest) {
		return h.lookupWithin(at), nil
	}
	// 晚于现存最晚帧：按外推上限处理。
	// 上限为 0 表示禁止外推：任何晚于最新帧的时间都直接报错。
	if at.Sub(newest) > h.cfg.maxExtrapolation {
		return Transform{}, ErrExtrapolation
	}
	return h.extrapolate(at), nil
}

// lookupWithin 处理 oldest <= at <= newest 的情况：命中帧直接返回，否则插值。
func (h *edgeHistory) lookupWithin(at time.Time) Transform {
	idx := sort.Search(len(h.entries), func(i int) bool {
		return !h.entries[i].ts.Before(at)
	})
	if idx < len(h.entries) && h.entries[idx].ts.Equal(at) {
		return h.entries[idx].tr
	}
	// at 严格落在两帧之间；at==oldest 的情况已由上面的相等分支覆盖。
	return interpolateTransform(h.entries[idx-1], h.entries[idx], at)
}

// extrapolate 沿最近一帧（或最近两帧）做线性保持/外推。
func (h *edgeHistory) extrapolate(at time.Time) Transform {
	if len(h.entries) < 2 {
		return h.entries[0].tr
	}
	a := h.entries[len(h.entries)-2]
	b := h.entries[len(h.entries)-1]
	if !b.ts.After(a.ts) {
		return b.tr
	}
	alpha := (at.Sub(a.ts).Seconds()) / b.ts.Sub(a.ts).Seconds()
	return Transform{
		Translation: a.tr.Translation.lerp(b.tr.Translation, alpha),
		Rotation:    a.tr.Rotation.slerp(b.tr.Rotation, alpha),
	}
}

func (h *edgeHistory) len() int { return len(h.entries) }

package transformtree

import (
	"errors"
	"testing"
	"time"
)

func translation(x float64) Transform {
	return Transform{Translation: Vector3{X: x}, Rotation: Quaternion{W: 1}}
}

func newTestHistory(age time.Duration, entries int, extra time.Duration) *edgeHistory {
	return newEdgeHistory(historyConfig{
		maxAge:           age,
		maxEntries:       entries,
		maxExtrapolation: extra,
	})
}

func TestHistoryOutOfOrderAndDuplicates(t *testing.T) {
	h := newTestHistory(0, 0, 0)
	base := time.Unix(100, 0)
	h.insert(translation(3), base.Add(3*time.Second))
	h.insert(translation(1), base.Add(1*time.Second)) // 乱序晚到
	h.insert(translation(2), base.Add(2*time.Second))
	h.insert(translation(2.5), base.Add(2*time.Second)) // 重复时间戳：必须忽略

	if h.len() != 3 {
		t.Fatalf("len = %d, want 3", h.len())
	}
	got, err := h.lookup(base.Add(2 * time.Second))
	if err != nil {
		t.Fatal(err)
	}
	if got.Translation.X != 2 {
		t.Fatalf("duplicate timestamp overwrote state: x = %v, want 2", got.Translation.X)
	}
}

func TestHistoryInterpolationBoundaries(t *testing.T) {
	h := newTestHistory(0, 0, 0)
	base := time.Unix(0, 0)
	h.insert(translation(0), base)
	h.insert(translation(10), base.Add(10*time.Second))

	// 精确命中首帧与末帧
	if tr, _ := h.lookup(base); tr.Translation.X != 0 {
		t.Fatalf("oldest frame x = %v, want 0", tr.Translation.X)
	}
	if tr, _ := h.lookup(base.Add(10 * time.Second)); tr.Translation.X != 10 {
		t.Fatalf("newest frame x = %v, want 10", tr.Translation.X)
	}
	// 中间插值
	if tr, _ := h.lookup(base.Add(3 * time.Second)); !approxEq(tr.Translation.X, 3, eps) {
		t.Fatalf("interpolated x = %v, want 3", tr.Translation.X)
	}
}

func TestHistoryEmptyAndExpired(t *testing.T) {
	h := newTestHistory(0, 0, 0)
	if _, err := h.lookup(time.Now()); !errors.Is(err, ErrNoData) {
		t.Fatalf("empty lookup err = %v, want ErrNoData", err)
	}

	base := time.Unix(200, 0)
	h.insert(translation(1), base)
	// 早于现存最早帧：已淘汰语义（过期），与外推错误可区分
	if _, err := h.lookup(base.Add(-time.Second)); !errors.Is(err, ErrExpired) {
		t.Fatalf("old-time lookup err = %v, want ErrExpired", err)
	}
}

func TestHistoryExtrapolation(t *testing.T) {
	h := newTestHistory(0, 0, 2*time.Second)
	base := time.Unix(0, 0)
	h.insert(translation(0), base)
	h.insert(translation(10), base.Add(10*time.Second))

	// 上限内：允许外推
	tr, err := h.lookup(base.Add(11 * time.Second))
	if err != nil {
		t.Fatal(err)
	}
	if !approxEq(tr.Translation.X, 11, eps) {
		t.Fatalf("extrapolated x = %v, want 11", tr.Translation.X)
	}
	// 恰好等于上限：允许
	if _, err := h.lookup(base.Add(12 * time.Second)); err != nil {
		t.Fatalf("limit boundary err = %v, want nil", err)
	}
	// 超出上限
	if _, err := h.lookup(base.Add(13 * time.Second)); !errors.Is(err, ErrExtrapolation) {
		t.Fatalf("beyond-limit err = %v, want ErrExtrapolation", err)
	}
}

func TestHistoryExtrapolationDisabled(t *testing.T) {
	h := newTestHistory(0, 0, 0)
	base := time.Unix(0, 0)
	h.insert(translation(0), base)
	if _, err := h.lookup(base.Add(time.Nanosecond)); !errors.Is(err, ErrExtrapolation) {
		t.Fatalf("err = %v, want ErrExtrapolation", err)
	}
}

func TestHistoryEvictionByAgeWithLateData(t *testing.T) {
	h := newTestHistory(10*time.Second, 0, 0)
	base := time.Unix(0, 0)
	h.insert(translation(20), base.Add(20*time.Second))
	h.insert(translation(15), base.Add(15*time.Second)) // 晚到，但仍落在窗口内

	if h.len() != 2 {
		t.Fatalf("len = %d, want 2", h.len())
	}
	if tr, _ := h.lookup(base.Add(15 * time.Second)); tr.Translation.X != 15 {
		t.Fatalf("late in-window x = %v, want 15", tr.Translation.X)
	}

	h.insert(translation(9), base.Add(9*time.Second)) // 严格早于窗口下界 10s：立即淘汰
	if h.len() != 2 {
		t.Fatalf("len = %d, want 2 after stale insert", h.len())
	}

	if _, err := h.lookup(base.Add(9 * time.Second)); !errors.Is(err, ErrExpired) {
		t.Fatalf("evicted frame err = %v, want ErrExpired", err)
	}
	if _, err := h.lookup(base.Add(10 * time.Second)); !errors.Is(err, ErrExpired) {
		t.Fatalf("below-window lookup err = %v, want ErrExpired", err)
	}
	if tr, _ := h.lookup(base.Add(15 * time.Second)); tr.Translation.X != 15 {
		t.Fatalf("in-window late frame x = %v, want 15", tr.Translation.X)
	}
	if tr, _ := h.lookup(base.Add(20 * time.Second)); tr.Translation.X != 20 {
		t.Fatalf("x = %v, want 20", tr.Translation.X)
	}
}

func TestHistoryEvictionByEntries(t *testing.T) {
	h := newTestHistory(0, 3, 0)
	base := time.Unix(0, 0)
	for i := 0; i < 5; i++ {
		h.insert(translation(float64(i)), base.Add(time.Duration(i)*time.Second))
	}
	if h.len() != 3 {
		t.Fatalf("len = %d, want 3", h.len())
	}
	if tr, _ := h.lookup(base.Add(2 * time.Second)); tr.Translation.X != 2 {
		t.Fatalf("oldest retained x = %v, want 2", tr.Translation.X)
	}
	if _, err := h.lookup(base.Add(time.Second)); !errors.Is(err, ErrExpired) {
		t.Fatalf("dropped frame err = %v, want ErrExpired", err)
	}
}

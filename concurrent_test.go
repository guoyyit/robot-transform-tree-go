package transformtree

import (
	"errors"
	"runtime"
	"sync"
	"testing"
	"time"
)

// TestConcurrentPublishLookup 在持续发布的同时并发查询：
// 查询只能看到完整一致的图快照，结果必须可重复、无 panic、无撕裂。
func TestConcurrentPublishLookup(t *testing.T) {
	tree := New(Config{
		MaxAge:           5 * time.Second,
		MaxEntries:       20,
		MaxExtrapolation: 50 * time.Millisecond,
	})

	const frames = 200
	start := time.Unix(0, 0)
	var publishWg sync.WaitGroup

	// 两个发布者，沿同一条链持续更新，顺序故意打乱。
	publishWg.Add(2)
	go func() {
		defer publishWg.Done()
		for i := frames - 1; i >= 0; i-- {
			tree.Publish("a", "b", translation(float64(i)),
				start.Add(time.Duration(i)*time.Millisecond))
		}
	}()
	go func() {
		defer publishWg.Done()
		for i := 0; i < frames; i++ {
			j := frames - 1 - i
			tree.Publish("b", "c", translation(float64(j)),
				start.Add(time.Duration(j)*time.Millisecond))
		}
	}()

	var queryWg sync.WaitGroup
	queryWg.Add(runtime.NumCPU())
	for w := 0; w < runtime.NumCPU(); w++ {
		go func() {
			defer queryWg.Done()
			for k := 0; k < 2000; k++ {
				at := start.Add(time.Duration(k%frames) * time.Millisecond)
				tr1, err1 := tree.LookupTransform("a", "c", at)
				if err1 != nil {
					// 发布完成前历史可能暂时为空/过期；这些是明确错误，不能撕裂。
					if !errors.Is(err1, ErrNoData) && !errors.Is(err1, ErrExpired) &&
						!errors.Is(err1, ErrExtrapolation) && !errors.Is(err1, ErrFrameNotFound) {
						t.Errorf("unexpected lookup error: %v", err1)
					}
					continue
				}
				// 同一时刻的重复查询必须一致。
				tr2, err2 := tree.LookupTransform("a", "c", at)
				if err2 != nil {
					t.Errorf("repeat query error: %v", err2)
					continue
				}
				if tr1.Translation.X != tr2.Translation.X {
					t.Errorf("torn read: %v vs %v at %v", tr1.Translation.X, tr2.Translation.X, at)
				}
			}
		}()
	}

	queryWg.Wait()
	publishWg.Wait()

	// 发布完成后，全链查询与逐级手工计算一致（取保留窗口内的时间点）。
	finalAt := start.Add(190 * time.Millisecond)
	tr, err := tree.LookupTransform("a", "c", finalAt)
	if err != nil {
		t.Fatal(err)
	}
	ab, _ := tree.LookupTransform("a", "b", finalAt)
	bc, _ := tree.LookupTransform("b", "c", finalAt)
	if !approxEq(tr.Translation.X, ab.Compose(bc).Translation.X, eps) {
		t.Fatalf("chain x = %v, want manual %v", tr.Translation.X, ab.Compose(bc).Translation.X)
	}
}

// TestHistoryBoundedMemory 长时间高频发布后，每条边的历史帧数必须有界。
func TestHistoryBoundedMemory(t *testing.T) {
	const maxEntries = 50
	tree := New(Config{
		MaxAge:           time.Hour, // 主要靠数量上限约束
		MaxEntries:       maxEntries,
		MaxExtrapolation: 0,
	})
	start := time.Unix(1_000_000, 0)
	for i := 0; i < 100_000; i++ {
		tree.Publish("a", "b", translation(float64(i)), start.Add(time.Duration(i)*time.Millisecond))
	}

	tree.mu.RLock()
	h := tree.edges[edgeKey{parent: "a", child: "b"}].history
	n := h.len()
	oldest := h.entries[0].ts
	tree.mu.RUnlock()

	if n != maxEntries {
		t.Fatalf("history len = %d, want %d", n, maxEntries)
	}
	// 最新一帧保留，最旧一帧对应已淘汰时间点的查询必须报“过期”。
	if x, _ := h.lookup(start.Add(99_999 * time.Millisecond)); x.Translation.X != 99_999 {
		t.Fatalf("newest frame x = %v, want 99999", x.Translation.X)
	}
	if _, err := h.lookup(oldest.Add(-time.Nanosecond)); !errors.Is(err, ErrExpired) {
		t.Fatalf("evicted-time err = %v, want ErrExpired", err)
	}
}

// TestConcurrentCycleToggle 并发地加入/移除闭环边，查询只可能成功或返回闭环错误。
func TestConcurrentCycleToggle(t *testing.T) {
	tree := New(Config{MaxAge: time.Hour, MaxEntries: 10})
	now := time.Unix(0, 0)
	tree.Publish("a", "b", translation(1), now)
	tree.Publish("b", "c", translation(2), now)

	var wg sync.WaitGroup
	wg.Add(2)
	go func() {
		defer wg.Done()
		for i := 0; i < 500; i++ {
			tree.Publish("c", "a", translation(4), now)
			tree.Remove("c", "a")
		}
	}()
	go func() {
		defer wg.Done()
		for i := 0; i < 500; i++ {
			tr, err := tree.LookupTransform("a", "c", now)
			switch {
			case errors.Is(err, ErrCycle):
			case err != nil:
				t.Errorf("unexpected err: %v", err)
			case tr.Translation.X != 3:
				t.Errorf("tr x = %v, want 3", tr.Translation.X)
			}
		}
	}()
	wg.Wait()
}

func TestRaceDetectorPasses(t *testing.T) {
	tree := New(Config{MaxAge: time.Second, MaxEntries: 100, MaxExtrapolation: time.Second})
	start := time.Now()
	var wg sync.WaitGroup
	wg.Add(3)
	go func() {
		defer wg.Done()
		for i := 0; i < 1000; i++ {
			tree.Publish("p", "q", translation(float64(i)), start.Add(time.Duration(i)*time.Millisecond))
		}
	}()
	go func() {
		defer wg.Done()
		for i := 0; i < 1000; i++ {
			tree.PublishStatic("root", "p", translation(0))
		}
	}()
	go func() {
		defer wg.Done()
		for i := 0; i < 1000; i++ {
			_, _ = tree.LookupTransform("root", "q", start.Add(time.Duration(i)*time.Millisecond))
		}
	}()
	wg.Wait()
}

package tftree

import (
	"errors"
	"sync"
	"testing"
	"time"
)

// TestConcurrentPublishAndLookup 在持续发布的同时并发查询，
// 验证不会产生撕裂结果或数据竞争（配合 -race 运行）。
func TestConcurrentPublishAndLookup(t *testing.T) {
	tree := NewTree(Options{
		MaxHistoryAge:    200 * time.Millisecond,
		MaxExtrapolation: time.Second,
		MaxSamples:       1000,
	})
	base := time.Time{}

	// 初始结构：world -> a -> b，以及静态分支 world -> fixed。
	tree.PublishDynamic("world", "a", base, translate(0, 0, 0))
	tree.PublishDynamic("a", "b", base, translate(0, 0, 0))
	tree.PublishStatic("world", "fixed", translate(7, 7, 0))

	var wg sync.WaitGroup
	stop := make(chan struct{})

	// 3 个发布者，各自以不同频率更新一条边。
	for g := 0; g < 3; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			tick := time.Duration(g+1) * time.Millisecond
			i := 1
			for {
				select {
				case <-stop:
					return
				default:
				}
				now := base.Add(time.Duration(i) * 5 * time.Millisecond)
				switch g {
				case 0:
					tree.PublishDynamic("world", "a", now, translate(float64(i), 0, 0))
				case 1:
					tree.PublishDynamic("a", "b", now, translate(0, float64(i), 0))
				case 2:
					// 乱序/重复：交替发更旧和相同时间戳，应被忽略。
					tree.PublishDynamic("a", "b", now.Add(-time.Hour), translate(999, 0, 0))
				}
				i++
				time.Sleep(tick)
			}
		}(g)
	}

	// 4 个查询者：查询只应返回 nil、时间语义错误或闭环错误，
	// 绝不应 panic。
	for g := 0; g < 4; g++ {
		wg.Add(1)
		go func(g int) {
			defer wg.Done()
			for {
				select {
				case <-stop:
					return
				default:
				}
				at := base.Add(time.Duration(g) * time.Millisecond)
				if _, err := tree.Lookup("world", "b", at); err != nil &&
					!errors.Is(err, ErrExpired) && !errors.Is(err, ErrExtrapolation) &&
					!errors.Is(err, ErrEmptyHistory) && !errors.Is(err, ErrCycleDetected) {
					t.Errorf("unexpected lookup error: %v", err)
					return
				}
				if _, err := tree.Lookup("fixed", "b", at); err != nil &&
					!errors.Is(err, ErrExpired) && !errors.Is(err, ErrExtrapolation) &&
					!errors.Is(err, ErrEmptyHistory) && !errors.Is(err, ErrCycleDetected) {
					t.Errorf("unexpected branch lookup error: %v", err)
					return
				}
			}
		}(g)
	}

	time.Sleep(300 * time.Millisecond)
	close(stop)
	wg.Wait()
}

// TestMemoryBoundedByTime 验证长时间高频发布后历史长度有界，
// 且查询已淘汰时间点返回可区分的 ErrExpired。
func TestMemoryBoundedByTime(t *testing.T) {
	const window = 50 * time.Millisecond
	tree := NewTree(Options{
		MaxHistoryAge:    window,
		MaxExtrapolation: 0,
	})
	base := time.Time{}

	totalFrames := 5000
	for i := 0; i < totalFrames; i++ {
		now := base.Add(time.Duration(i) * time.Millisecond)
		if err := tree.PublishDynamic("world", "a", now, translate(float64(i), 0, 0)); err != nil {
			t.Fatal(err)
		}
	}

	tree.mu.RLock()
	e := tree.edges[edgeKey("world", "a")]
	length := len(e.history)
	oldest := e.history[0].Timestamp
	newest := e.history[len(e.history)-1].Timestamp
	tree.mu.RUnlock()

	if newest.Sub(oldest) > window {
		t.Fatalf("retained span = %v, exceeds window %v", newest.Sub(oldest), window)
	}
	if length > int(window/time.Millisecond)+2 {
		t.Fatalf("history length = %d, expected bounded to about %d frames",
			length, int(window/time.Millisecond)+1)
	}

	// 已淘汰时间点：可区分的过期错误。
	if _, err := tree.Lookup("world", "a", base); !errors.Is(err, ErrExpired) {
		t.Fatalf("expired query err = %v, want ErrExpired", err)
	}
	// 最新时间点仍可精确查询。
	pose, err := tree.Lookup("world", "a", newest)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, pose.Translation.X, float64(totalFrames-1))
}

// TestMemoryBoundedBySamples 验证 MaxSamples 条数上限。
func TestMemoryBoundedBySamples(t *testing.T) {
	tree := NewTree(Options{
		MaxHistoryAge:    time.Hour,
		MaxExtrapolation: 0,
		MaxSamples:       10,
	})
	base := time.Time{}
	for i := 0; i < 1000; i++ {
		if err := tree.PublishDynamic("world", "a",
			base.Add(time.Duration(i)*time.Millisecond), translate(float64(i), 0, 0)); err != nil {
			t.Fatal(err)
		}
	}
	tree.mu.RLock()
	length := len(tree.edges[edgeKey("world", "a")].history)
	tree.mu.RUnlock()
	if length != 10 {
		t.Fatalf("history length = %d, want 10", length)
	}
}

// TestConcurrentSameSnapshot 验证同一时间点的并发查询结果一致。
// 先构建静态快照（发布完成后不再写入），再用屏障同时发起查询，
// 确认读路径在并发下返回同一份确定结果。
func TestConcurrentSameSnapshot(t *testing.T) {
	tree := NewTree(Options{MaxHistoryAge: time.Hour, MaxExtrapolation: time.Minute})
	base := time.Time{}
	tree.PublishDynamic("world", "a", base, translate(2, 0, 0))
	tree.PublishDynamic("a", "b", base, translate(0, 3, 0))

	const readers = 16
	results := make([]Transform, readers)
	errs := make([]error, readers)

	start := make(chan struct{})
	var wg sync.WaitGroup
	wg.Add(readers)
	for i := 0; i < readers; i++ {
		go func(i int) {
			defer wg.Done()
			<-start
			results[i], errs[i] = tree.Lookup("world", "b", base)
		}(i)
	}
	close(start)
	wg.Wait()

	for i := 1; i < readers; i++ {
		if errs[i] != nil {
			t.Fatalf("reader %d error: %v", i, errs[i])
		}
		if results[i] != results[0] {
			t.Fatalf("reader %d = %+v, want %+v", i, results[i], results[0])
		}
	}
	approxVec(t, results[0].Translation, Vector3{X: 2, Y: 3})
}

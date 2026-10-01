package tftree

import (
	"errors"
	"testing"
	"time"
)

func TestCycleDetectedAndRecovered(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)
	tree.PublishDynamic("a", "b", base, translate(1, 0, 0))
	tree.PublishDynamic("b", "c", base, translate(2, 0, 0))

	// 正常链 a->b->c 可查。
	if _, err := tree.Lookup("a", "c", base); err != nil {
		t.Fatalf("before cycle: %v", err)
	}

	// 再加一条 c->a，使 a 向上遍历时遇到 a->b->c->a 的有向环。
	if err := tree.PublishDynamic("c", "a", base, translate(3, 0, 0)); err != nil {
		t.Fatal(err)
	}
	if _, err := tree.Lookup("a", "c", base); !errors.Is(err, ErrCycleDetected) {
		t.Fatalf("during cycle err = %v, want ErrCycleDetected", err)
	}
	// 环上查询任意图上帧都必须终止。
	if _, err := tree.Lookup("b", "c", base); !errors.Is(err, ErrCycleDetected) {
		t.Fatalf("cycle path err = %v, want ErrCycleDetected", err)
	}

	// 移除环边后自动恢复。
	tree.RemoveTransform("c", "a")
	pose, err := tree.Lookup("a", "c", base)
	if err != nil {
		t.Fatalf("after removal: %v", err)
	}
	approxFloat(t, pose.Translation.X, 3)
}

func TestSelfReferentialRejected(t *testing.T) {
	tree := newTestTree()
	if err := tree.PublishDynamic("a", "a", time.Now(), translate(0, 0, 0)); !errors.Is(err, ErrCycleDetected) {
		t.Fatalf("dynamic err = %v, want ErrCycleDetected", err)
	}
	if err := tree.PublishStatic("a", "a", translate(0, 0, 0)); !errors.Is(err, ErrCycleDetected) {
		t.Fatalf("static err = %v, want ErrCycleDetected", err)
	}
}

func TestTreeLevelHistoryErrors(t *testing.T) {
	tree := NewTree(Options{
		MaxHistoryAge:    time.Second,
		MaxExtrapolation: 100 * time.Millisecond,
	})
	t0 := time.Time{}

	tree.PublishDynamic("world", "a", t0, translate(0, 0, 0))
	tree.PublishDynamic("world", "a", t0.Add(2*time.Second), translate(20, 0, 0))
	tree.PublishDynamic("a", "b", t0, translate(0, 0, 0))
	tree.PublishDynamic("a", "b", t0.Add(2*time.Second), translate(0, 0, 0))

	// t0 已随第二帧发布被淘汰（窗口 1s，cutoff=1s）。
	if _, err := tree.Lookup("world", "a", t0); !errors.Is(err, ErrExpired) {
		t.Fatalf("expired err = %v, want ErrExpired", err)
	}
	// 未来超出外推上限。
	future := t0.Add(2*time.Second + 500*time.Millisecond)
	if _, err := tree.Lookup("world", "a", future); !errors.Is(err, ErrExtrapolation) {
		t.Fatalf("extrapolation err = %v, want ErrExtrapolation", err)
	}
	// 链中任意一条边超范围都会向上传播错误。
	tree.PublishDynamic("a", "b", t0.Add(2*time.Second), translate(0, 0, 0))
	if _, err := tree.Lookup("world", "b", future); !errors.Is(err, ErrExtrapolation) {
		t.Fatalf("chain extrapolation err = %v, want ErrExtrapolation", err)
	}
}

func TestMultipleIncomingEdges(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)
	tree.PublishDynamic("world", "a", base, translate(1, 0, 0))
	tree.PublishDynamic("a", "c", base, translate(2, 0, 0))

	// c 同时再挂到 world 下：两条入边并存，两条路径都应可查。
	tree.PublishDynamic("world", "c", base, translate(5, 0, 0))
	viaA, err := tree.Lookup("a", "c", base)
	if err != nil {
		t.Fatalf("via a: %v", err)
	}
	approxFloat(t, viaA.Translation.X, 2)

	direct, err := tree.Lookup("world", "c", base)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, direct.Translation.X, 5)

	// 移除直连边后只能走经 a 的路径。
	tree.RemoveTransform("world", "c")
	viaWorld, err := tree.Lookup("world", "c", base)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, viaWorld.Translation.X, 3)
}

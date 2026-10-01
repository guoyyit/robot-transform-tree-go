package transformtree

import (
	"errors"
	"testing"
	"time"
)

func TestDirectDynamicEdge(t *testing.T) {
	tree := New(Config{MaxAge: time.Minute, MaxEntries: 100, MaxExtrapolation: 0})
	now := time.Unix(1000, 0)
	tree.Publish("a", "b", translation(5), now)

	tr, err := tree.LookupTransform("a", "b", now)
	if err != nil {
		t.Fatal(err)
	}
	if tr.Translation.X != 5 {
		t.Fatalf("a->b x = %v, want 5", tr.Translation.X)
	}

	// 反方向取逆
	inv, err := tree.LookupTransform("b", "a", now)
	if err != nil {
		t.Fatal(err)
	}
	if !approxEq(inv.Translation.X, -5, eps) {
		t.Fatalf("b->a x = %v, want -5", inv.Translation.X)
	}
}

func TestChainResolution(t *testing.T) {
	tree := New(Config{MaxAge: time.Minute, MaxEntries: 100})
	now := time.Unix(1000, 0)
	tree.Publish("world", "a", translation(1), now)
	tree.Publish("a", "b", translation(2), now)
	tree.Publish("b", "c", translation(3), now)

	// world <- a <- b <- c：逐级手工拼接结果为 1+2+3
	tr, err := tree.LookupTransform("world", "c", now)
	if err != nil {
		t.Fatal(err)
	}
	if !approxEq(tr.Translation.X, 6, eps) {
		t.Fatalf("world->c x = %v, want 6", tr.Translation.X)
	}

	// 部分链与反向链
	if tr, _ := tree.LookupTransform("a", "c", now); !approxEq(tr.Translation.X, 5, eps) {
		t.Fatalf("a->c x = %v, want 5", tr.Translation.X)
	}
	if tr, _ := tree.LookupTransform("c", "world", now); !approxEq(tr.Translation.X, -6, eps) {
		t.Fatalf("c->world x = %v, want -6", tr.Translation.X)
	}
}

func TestChainWithRotation(t *testing.T) {
	tree := New(Config{MaxAge: time.Minute, MaxEntries: 100})
	now := time.Unix(1000, 0)
	rotZ90 := Transform{
		Translation: Vector3{X: 1},
		Rotation:    Quaternion{Z: 0.7071067811865475, W: 0.7071067811865476},
	}
	tree.Publish("world", "a", rotZ90, now)
	tree.Publish("a", "b", translation(1), now)

	// world<-a 先把 a 原点平移到 (1,0) 并旋转 90°；
	// a<-b 又平移 (1,0)，经旋转后变为 (0,1)，故 world<-b 平移为 (1,1)。
	tr, err := tree.LookupTransform("world", "b", now)
	if err != nil {
		t.Fatal(err)
	}
	vecApprox(t, tr.Translation, Vector3{X: 1, Y: 1})
}

func TestStaticTransforms(t *testing.T) {
	tree := New(Config{})
	tree.PublishStatic("world", "map", translation(7))
	tree.Publish("map", "sensor", translation(1), time.Unix(0, 0))

	// 静态边不参与时间语义，任意时间查询都一致
	for _, at := range []time.Time{time.Unix(-100, 0), time.Unix(0, 0), time.Unix(1e6, 0)} {
		tr, err := tree.LookupTransform("world", "map", at)
		if err != nil {
			t.Fatal(err)
		}
		if tr.Translation.X != 7 {
			t.Fatalf("static x = %v, want 7", tr.Translation.X)
		}
	}

	// 静态边与动态边混合拼接
	tr, err := tree.LookupTransform("world", "sensor", time.Unix(0, 0))
	if err != nil {
		t.Fatal(err)
	}
	if tr.Translation.X != 8 {
		t.Fatalf("world->sensor x = %v, want 8", tr.Translation.X)
	}
}

func TestArrivalOrderIndependence(t *testing.T) {
	base := time.Unix(0, 0)
	inOrder := func() float64 {
		tree := New(Config{MaxAge: time.Hour, MaxEntries: 100})
		for i := 0; i <= 10; i++ {
			tree.Publish("a", "b", translation(float64(i)), base.Add(time.Duration(i)*time.Second))
		}
		tr, _ := tree.LookupTransform("a", "b", base.Add(5500*time.Millisecond))
		return tr.Translation.X
	}
	outOfOrder := func() float64 {
		tree := New(Config{MaxAge: time.Hour, MaxEntries: 100})
		order := []int{10, 3, 7, 0, 9, 2, 5, 1, 8, 4, 6}
		for _, i := range order {
			tree.Publish("a", "b", translation(float64(i)), base.Add(time.Duration(i)*time.Second))
		}
		// 重复发布同一帧不得改变状态
		tree.Publish("a", "b", translation(999), base.Add(5*time.Second))
		tr, _ := tree.LookupTransform("a", "b", base.Add(5500*time.Millisecond))
		return tr.Translation.X
	}
	if x1, x2 := inOrder(), outOfOrder(); !approxEq(x1, x2, eps) || !approxEq(x1, 5.5, eps) {
		t.Fatalf("in-order x=%v out-of-order x=%v, want 5.5", x1, x2)
	}
}

func TestFrameErrors(t *testing.T) {
	tree := New(Config{})
	if _, err := tree.LookupTransform("a", "b", time.Now()); !errors.Is(err, ErrFrameNotFound) {
		t.Fatalf("missing frames err = %v, want ErrFrameNotFound", err)
	}

	tree.PublishStatic("a", "b", translation(1))
	tree.PublishStatic("c", "d", translation(2))
	if _, err := tree.LookupTransform("a", "d", time.Now()); !errors.Is(err, ErrFrameNotFound) {
		t.Fatalf("disconnected err = %v, want ErrFrameNotFound", err)
	}
}

func TestDynamicEdgeErrors(t *testing.T) {
	tree := New(Config{MaxAge: time.Minute, MaxEntries: 100, MaxExtrapolation: time.Second})
	base := time.Unix(500, 0)
	tree.Publish("a", "b", translation(0), base)
	tree.Publish("a", "b", translation(10), base.Add(10*time.Second))

	if _, err := tree.LookupTransform("a", "b", base.Add(-time.Nanosecond)); !errors.Is(err, ErrExpired) {
		t.Fatalf("expired err = %v, want ErrExpired", err)
	}
	if _, err := tree.LookupTransform("a", "b", base.Add(12*time.Second)); !errors.Is(err, ErrExtrapolation) {
		t.Fatalf("extrapolation err = %v, want ErrExtrapolation", err)
	}
	if _, err := tree.LookupTransform("a", "b", base.Add(11*time.Second)); err != nil {
		t.Fatalf("extrapolation at limit boundary err = %v, want nil", err)
	}
	if tr, _ := tree.LookupTransform("a", "b", base.Add(5*time.Second)); !approxEq(tr.Translation.X, 5, eps) {
		t.Fatalf("interpolated x = %v, want 5", tr.Translation.X)
	}
}

func TestCycleDetectionAndRecovery(t *testing.T) {
	tree := New(Config{MaxAge: time.Hour, MaxEntries: 100})
	now := time.Unix(0, 0)
	tree.Publish("a", "b", translation(1), now)
	tree.Publish("b", "c", translation(2), now)

	if tr, err := tree.LookupTransform("a", "c", now); err != nil || tr.Translation.X != 3 {
		t.Fatalf("pre-cycle tr=%v err=%v", tr, err)
	}

	// 加入 c -> a 构成闭环
	tree.Publish("c", "a", translation(4), now)
	if _, err := tree.LookupTransform("a", "c", now); !errors.Is(err, ErrCycle) {
		t.Fatalf("with cycle err = %v, want ErrCycle", err)
	}
	if _, err := tree.LookupTransform("a", "b", now); !errors.Is(err, ErrCycle) {
		t.Fatalf("cycle affects component err = %v, want ErrCycle", err)
	}

	// 移除闭环边后自动恢复
	tree.Remove("c", "a")
	if tr, err := tree.LookupTransform("a", "c", now); err != nil || tr.Translation.X != 3 {
		t.Fatalf("post-cycle tr=%v err=%v", tr, err)
	}
}

func TestSelfLoopAndParallelEdgesAreCycles(t *testing.T) {
	tree := New(Config{})
	tree.PublishStatic("a", "a", translation(1))
	if _, err := tree.LookupTransform("a", "a", time.Now()); !errors.Is(err, ErrCycle) {
		t.Fatalf("self-loop err = %v, want ErrCycle", err)
	}
	tree.Remove("a", "a")

	tree2 := New(Config{})
	tree2.PublishStatic("a", "b", translation(1))
	tree2.PublishStatic("b", "a", translation(-1))
	if _, err := tree2.LookupTransform("a", "b", time.Now()); !errors.Is(err, ErrCycle) {
		t.Fatalf("parallel edges err = %v, want ErrCycle", err)
	}
}

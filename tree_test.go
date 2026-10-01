package tftree

import (
	"errors"
	"math"
	"testing"
	"time"
)

func translate(x, y, z float64) Transform {
	return Transform{Translation: Vector3{X: x, Y: y, Z: z}, Rotation: Quaternion{W: 1}}
}

func newTestTree() *Tree {
	return NewTree(Options{
		MaxHistoryAge:    time.Hour,
		MaxExtrapolation: time.Minute,
	})
}

func TestLookupDirectEdgeAndInverse(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)
	if err := tree.PublishDynamic("world", "a", base, translate(3, 4, 0)); err != nil {
		t.Fatal(err)
	}

	// world -> a 的点应平移 (3,4)。
	pose, err := tree.Lookup("world", "a", base)
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, pose.Translation, Vector3{X: 3, Y: 4})

	// 反查 a -> world 应为 (-3,-4)。
	inv, err := tree.Lookup("a", "world", base)
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, inv.Translation, Vector3{X: -3, Y: -4})

	// 同帧查询为单位变换。
	self, err := tree.Lookup("a", "a", base)
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, self.Translation, Vector3{})
	approxFloat(t, self.Rotation.W, 1)
}

func TestLookupChainMatchesManual(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)

	// world -> a (1,0,0)，a -> b (2,0,0)，b -> c (3,0,0)。
	if err := tree.PublishDynamic("world", "a", base, translate(1, 0, 0)); err != nil {
		t.Fatal(err)
	}
	if err := tree.PublishDynamic("a", "b", base, translate(2, 0, 0)); err != nil {
		t.Fatal(err)
	}
	if err := tree.PublishDynamic("b", "c", base, translate(3, 0, 0)); err != nil {
		t.Fatal(err)
	}

	// world -> c = 1+2+3 = 6。
	got, err := tree.Lookup("world", "c", base)
	if err != nil {
		t.Fatal(err)
	}

	ab := translate(1, 0, 0)
	bc := translate(2, 0, 0)
	cd := translate(3, 0, 0)
	want := ab.compose(bc).compose(cd)
	approxVec(t, got.Translation, want.Translation)
	approxFloat(t, got.Translation.X, 6)

	// c -> world = -6。
	back, err := tree.Lookup("c", "world", base)
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, back.Translation, Vector3{X: -6})

	// 反向链中点：a -> c = 2+3 = 5，b -> a = -2。
	mid, err := tree.Lookup("a", "c", base)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, mid.Translation.X, 5)
	up, err := tree.Lookup("b", "a", base)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, up.Translation.X, -2)
}

func TestLookupBranchingCommonAncestor(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)
	// world -> l -> l1 (10,0,0)，world -> r -> r1 (0,5,0)。
	tree.PublishDynamic("world", "l", base, translate(0, 0, 0))
	tree.PublishDynamic("l", "l1", base, translate(10, 0, 0))
	tree.PublishDynamic("world", "r", base, translate(0, 0, 0))
	tree.PublishDynamic("r", "r1", base, translate(0, 5, 0))

	// l1 -> r1：先 l1->world (-10,0)，再 world->r1 (0,5)，得 (-10,5)。
	pose, err := tree.Lookup("l1", "r1", base)
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, pose.Translation, Vector3{X: -10, Y: 5})
}

func TestLookupInterpolationAcrossChain(t *testing.T) {
	tree := newTestTree()
	t0 := time.Time{}
	tree.PublishDynamic("world", "a", t0, translate(0, 0, 0))
	tree.PublishDynamic("world", "a", t0.Add(10*time.Second), translate(10, 0, 0))
	tree.PublishDynamic("a", "b", t0, translate(0, 0, 0))
	tree.PublishDynamic("a", "b", t0.Add(10*time.Second), translate(0, 20, 0))

	at := t0.Add(5 * time.Second)
	pose, err := tree.Lookup("world", "b", at)
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, pose.Translation, Vector3{X: 5, Y: 10})
}

func TestLookupWithRotation(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)
	// world -> a：绕 z 旋转 90°。
	rot := Transform{
		Translation: Vector3{X: 1, Y: 0, Z: 0},
		Rotation:    Quaternion{Z: math.Sin(math.Pi / 4), W: math.Cos(math.Pi / 4)},
	}
	tree.PublishDynamic("world", "a", base, rot)
	// a -> b：在 a 系中沿 a 的 x 方向偏移 2。
	tree.PublishDynamic("a", "b", base, translate(2, 0, 0))

	// b 原点在 world 中 = (1,0) + R90*(2,0) = (1,2)。
	pose, err := tree.Lookup("world", "b", base)
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, pose.Translation, Vector3{X: 1, Y: 2})
}

func TestLookupStaticTransforms(t *testing.T) {
	tree := newTestTree()
	tree.PublishStatic("map", "odom", translate(100, 0, 0))
	tree.PublishStatic("odom", "base", translate(1, 0, 0))
	base := time.Time{}.Add(time.Second)

	// 静态边与查询时间无关；动态边仍受其外推窗口约束。
	pose, err := tree.Lookup("map", "base", base)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, pose.Translation.X, 101)

	// 纯静态链任意时间都可查。
	tree.PublishStatic("base", "sensor", translate(0, 1, 0))
	pose, err = tree.Lookup("map", "sensor", time.Time{})
	if err != nil {
		t.Fatal(err)
	}
	approxVec(t, pose.Translation, Vector3{X: 101, Y: 1})
}

func TestLookupMissingAndDisconnected(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)
	tree.PublishDynamic("world", "a", base, translate(1, 0, 0))
	tree.PublishDynamic("mars", "b", base, translate(2, 0, 0))

	if _, err := tree.Lookup("world", "ghost", base); !errors.Is(err, ErrFrameNotFound) {
		t.Fatalf("err = %v, want ErrFrameNotFound", err)
	}
	if _, err := tree.Lookup("ghost", "world", base); !errors.Is(err, ErrFrameNotFound) {
		t.Fatalf("err = %v, want ErrFrameNotFound", err)
	}
	if _, err := tree.Lookup("a", "b", base); !errors.Is(err, ErrNoConnection) {
		t.Fatalf("err = %v, want ErrNoConnection", err)
	}
}

func TestDynamicPublishReplacesStatic(t *testing.T) {
	tree := newTestTree()
	base := time.Time{}.Add(time.Second)
	if err := tree.PublishStatic("world", "a", translate(7, 0, 0)); err != nil {
		t.Fatal(err)
	}
	if err := tree.PublishDynamic("world", "a", base, translate(9, 0, 0)); err != nil {
		t.Fatal(err)
	}
	pose, err := tree.Lookup("world", "a", base)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, pose.Translation.X, 9)
}

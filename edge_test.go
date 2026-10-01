package tftree

import (
	"errors"
	"testing"
	"time"
)

func sampleFrame(sec float64, x float64) TimestampedTransform {
	return TimestampedTransform{
		Timestamp: time.Time{}.Add(time.Duration(sec * float64(time.Second))),
		Transform: Transform{Translation: Vector3{X: x}, Rotation: Quaternion{W: 1}},
	}
}

func atMS(ms int64) time.Time { return time.Time{}.Add(time.Duration(ms) * time.Millisecond) }

func TestEdgeOutOfOrderAndDuplicate(t *testing.T) {
	e := newEdge("p", "c", false)
	// 乱序到达：1 -> 3 -> 2(旧，忽略) -> 3(重复，忽略)。
	if !e.add(sampleFrame(1, 10)) {
		t.Fatal("first add should succeed")
	}
	if !e.add(sampleFrame(3, 30)) {
		t.Fatal("newer add should succeed")
	}
	if e.add(sampleFrame(2, 999)) {
		t.Fatal("late older frame must be ignored")
	}
	if e.add(sampleFrame(3, 888)) {
		t.Fatal("duplicate timestamp must be ignored")
	}
	if len(e.history) != 2 {
		t.Fatalf("history len = %d, want 2", len(e.history))
	}
	// 中间没有第 2 秒帧，说明旧数据未插入。
	pose, err := e.sample(atMS(2000), time.Hour)
	if err != nil {
		t.Fatal(err)
	}
	approxFloat(t, pose.Translation.X, 20)
	// 第 3 秒的值保持 30，未被重复发布的 888 覆盖。
	pose, _ = e.sample(atMS(3000), time.Hour)
	approxFloat(t, pose.Translation.X, 30)
}

func TestEdgeInterpolationBoundaries(t *testing.T) {
	e := newEdge("p", "c", false)
	e.add(sampleFrame(0, 0))
	e.add(sampleFrame(10, 100))

	// 精确命中端点。
	pose, _ := e.sample(atMS(0), 0)
	approxFloat(t, pose.Translation.X, 0)
	pose, _ = e.sample(atMS(10000), 0)
	approxFloat(t, pose.Translation.X, 100)

	// 中间线性插值。
	pose, _ = e.sample(atMS(2500), 0)
	approxFloat(t, pose.Translation.X, 25)
	pose, _ = e.sample(atMS(7500), 0)
	approxFloat(t, pose.Translation.X, 75)
}

func TestEdgeEmptyHistory(t *testing.T) {
	e := newEdge("p", "c", false)
	if _, err := e.sample(time.Time{}, time.Minute); !errors.Is(err, ErrEmptyHistory) {
		t.Fatalf("err = %v, want ErrEmptyHistory", err)
	}
}

func TestEdgeExtrapolationFuture(t *testing.T) {
	e := newEdge("p", "c", false)
	e.add(sampleFrame(0, 0))
	e.add(sampleFrame(1, 10))

	// 允许外推 100ms。
	pose, err := e.sample(atMS(1050), 100*time.Millisecond)
	if err != nil || pose.Translation.X != 10 {
		t.Fatalf("within limit: pose=%v err=%v", pose, err)
	}
	// 超过上限。
	if _, err := e.sample(atMS(1500), 100*time.Millisecond); !errors.Is(err, ErrExtrapolation) {
		t.Fatalf("err = %v, want ErrExtrapolation", err)
	}
}

func TestEdgeExtrapolationPastBeforeAnyPrune(t *testing.T) {
	e := newEdge("p", "c", false)
	e.add(sampleFrame(2, 20))

	// 从未淘汰过：早于最早帧且超过外推上限 -> 外推错误（而非过期）。
	if _, err := e.sample(time.Time{}, 0); !errors.Is(err, ErrExtrapolation) {
		t.Fatalf("err = %v, want ErrExtrapolation", err)
	}
	// 外推范围内：保持最早帧的值。
	pose, err := e.sample(atMS(1500), time.Second)
	if err != nil || pose.Translation.X != 20 {
		t.Fatalf("pose=%v err=%v", pose, err)
	}
}

func TestEdgeExpiredAfterPrune(t *testing.T) {
	e := newEdge("p", "c", false)
	for s := float64(0); s <= 10; s++ {
		e.add(sampleFrame(s, s))
	}
	// 淘汰 5s 之前的帧。
	removed := e.prune(atMS(5000))
	if removed != 5 {
		t.Fatalf("removed = %d, want 5", removed)
	}
	if _, err := e.sample(atMS(4000), time.Hour); !errors.Is(err, ErrExpired) {
		t.Fatalf("err = %v, want ErrExpired", err)
	}
	// 仍在历史中的时间点可查。
	pose, err := e.sample(atMS(7000), 0)
	if err != nil || pose.Translation.X != 7 {
		t.Fatalf("pose=%v err=%v", pose, err)
	}
}

func TestEdgeCapHistory(t *testing.T) {
	e := newEdge("p", "c", false)
	for s := float64(0); s < 10; s++ {
		e.add(sampleFrame(s, s))
	}
	e.capHistory(3)
	if len(e.history) != 3 {
		t.Fatalf("len = %d, want 3", len(e.history))
	}
	approxFloat(t, e.history[0].Transform.Translation.X, 7)
	// 条数淘汰也算过期。
	if _, err := e.sample(time.Time{}, time.Hour); !errors.Is(err, ErrExpired) {
		t.Fatalf("err = %v, want ErrExpired", err)
	}
}

func TestEdgeStatic(t *testing.T) {
	e := newEdge("p", "c", true)
	e.staticPose = Transform{Translation: Vector3{X: 42}, Rotation: Quaternion{W: 1}}
	for _, at := range []time.Time{atMS(0), atMS(1000), atMS(9999)} {
		pose, err := e.sample(at, 0)
		if err != nil || pose.Translation.X != 42 {
			t.Fatalf("static pose at %v = %v, %v", at, pose, err)
		}
	}
}

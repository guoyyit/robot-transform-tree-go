package tftree

import (
	"math"
	"testing"
)

func approxFloat(t *testing.T, got, want float64) {
	t.Helper()
	if math.Abs(got-want) > 1e-9 {
		t.Fatalf("got %.9f, want %.9f", got, want)
	}
}

func approxVec(t *testing.T, got, want Vector3) {
	t.Helper()
	approxFloat(t, got.X, want.X)
	approxFloat(t, got.Y, want.Y)
	approxFloat(t, got.Z, want.Z)
}

func TestInverseTranslation(t *testing.T) {
	tr := Transform{Translation: Vector3{X: 1, Y: 2, Z: 3}, Rotation: Quaternion{W: 1}}
	got := tr.inverse().compose(tr)
	approxVec(t, got.Translation, Vector3{})
	approxFloat(t, got.Rotation.W, 1)
}

func TestRotationZ90(t *testing.T) {
	// 绕 Z 轴旋转 90°：x 轴单位向量应转到 y 轴。
	q := Quaternion{Z: math.Sin(math.Pi / 4), W: math.Cos(math.Pi / 4)}
	rotated := quatRotate(q, Vector3{X: 1})
	approxVec(t, rotated, Vector3{Y: 1})
}

func TestInverseRotation(t *testing.T) {
	q := Quaternion{
		X: 0.128, Y: 0.247, Z: 0.368, W: 0.886,
	}
	q = normalizeQuat(q)
	tr := Transform{Translation: Vector3{X: -2, Y: 0.5, Z: 4}, Rotation: q}
	roundTrip := tr.inverse().compose(tr)
	approxVec(t, roundTrip.Translation, Vector3{})
	approxFloat(t, roundTrip.Rotation.W, 1)
	approxFloat(t, roundTrip.Rotation.X, 0)
	approxFloat(t, roundTrip.Rotation.Y, 0)
	approxFloat(t, roundTrip.Rotation.Z, 0)
}

func TestSlerpEndpoints(t *testing.T) {
	t0 := TimestampedTransform{Transform: Transform{
		Translation: Vector3{X: 0},
		Rotation:    Quaternion{W: 1},
	}}
	t1 := TimestampedTransform{Transform: Transform{
		Translation: Vector3{X: 10},
		Rotation:    Quaternion{Z: math.Sin(math.Pi / 4), W: math.Cos(math.Pi / 4)},
	}}

	at0 := interpolate(t0, t1, 0)
	approxVec(t, at0.Translation, Vector3{})
	approxFloat(t, at0.Rotation.W, 1)

	at1 := interpolate(t0, t1, 1)
	approxVec(t, at1.Translation, Vector3{X: 10})
	approxFloat(t, at1.Rotation.Z, t1.Transform.Rotation.Z)
}

func TestSlerpMidpoint(t *testing.T) {
	// 0° 到 90° 的中点应为 45°。
	q0 := Quaternion{W: 1}
	q1 := Quaternion{Z: math.Sin(math.Pi / 4), W: math.Cos(math.Pi / 4)}
	t0 := TimestampedTransform{Transform: Transform{Rotation: q0}}
	t1 := TimestampedTransform{Transform: Transform{Rotation: q1}}
	mid := interpolate(t0, t1, 0.5)
	approxFloat(t, mid.Rotation.Z, math.Sin(math.Pi/8))
	approxFloat(t, mid.Rotation.W, math.Cos(math.Pi/8))

	// 中点旋转应把 x 轴转到 45° 方向。
	rotated := quatRotate(mid.Rotation, Vector3{X: 1})
	approxVec(t, rotated, Vector3{X: math.Cos(math.Pi / 4), Y: math.Sin(math.Pi / 4)})
}

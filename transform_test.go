package transformtree

import (
	"math"
	"testing"
	"time"
)

const eps = 1e-9

func approxEq(a, b, tol float64) bool {
	d := a - b
	if d < 0 {
		d = -d
	}
	return d <= tol
}

func vecApprox(t *testing.T, got, want Vector3) {
	t.Helper()
	if !approxEq(got.X, want.X, eps) || !approxEq(got.Y, want.Y, eps) || !approxEq(got.Z, want.Z, eps) {
		t.Fatalf("vector = %+v, want %+v", got, want)
	}
}

func TestQuaternionRotateZ90(t *testing.T) {
	q := Quaternion{Z: math.Sin(math.Pi / 4), W: math.Cos(math.Pi / 4)}
	vecApprox(t, q.rotate(Vector3{X: 1}), Vector3{Y: 1})
}

func TestInverseComposeRoundTrip(t *testing.T) {
	tr := Transform{
		Translation: Vector3{X: 1, Y: 2, Z: 3},
		Rotation:    Quaternion{Z: math.Sin(math.Pi / 8), W: math.Cos(math.Pi / 8)},
	}
	got := tr.Inverse().Compose(tr)
	vecApprox(t, got.Translation, Vector3{})
	if !approxEq(math.Abs(got.Rotation.W), 1, eps) {
		t.Fatalf("rotation = %+v, want identity", got.Rotation)
	}
}

func TestComposeTranslations(t *testing.T) {
	a := Transform{Translation: Vector3{X: 1}, Rotation: Quaternion{W: 1}}
	b := Transform{Translation: Vector3{X: 2}, Rotation: Quaternion{W: 1}}
	vecApprox(t, a.Compose(b).Translation, Vector3{X: 3})
}

func TestInterpolateTransform(t *testing.T) {
	t0 := time.Unix(0, 0)
	a := stampedTransform{ts: t0, tr: Transform{Translation: Vector3{X: 0}, Rotation: Quaternion{W: 1}}}
	b := stampedTransform{ts: t0.Add(2 * time.Second), tr: Transform{Translation: Vector3{X: 10}, Rotation: Quaternion{W: 1}}}
	got := interpolateTransform(a, b, t0.Add(time.Second))
	vecApprox(t, got.Translation, Vector3{X: 5})
}

func TestSlerpHalfwayZ(t *testing.T) {
	// 0 -> 90 度的一半应为 45 度
	q0 := Quaternion{W: 1}
	q1 := Quaternion{Z: math.Sin(math.Pi / 4), W: math.Cos(math.Pi / 4)}
	qm := q0.slerp(q1, 0.5)
	want := math.Pi / 4
	if angle := 2 * math.Acos(qm.W); !approxEq(angle, want, 1e-8) {
		t.Fatalf("angle = %v, want %v", angle, want)
	}
}

package transformtree

import (
	"math"
	"time"
)

// Vector3 是三维平移向量。
type Vector3 struct {
	X, Y, Z float64
}

// Quaternion 是单位四元数（W 为实部）。
type Quaternion struct {
	X, Y, Z, W float64
}

// Transform 描述一个坐标系相对其父坐标系的位姿：
// 把子坐标系中的点变换到父坐标系。
type Transform struct {
	Translation Vector3
	Rotation    Quaternion
}

// stampedTransform 是带时间戳的一帧动态变换采样。
type stampedTransform struct {
	ts time.Time
	tr Transform
}

// Identity 返回单位变换。
func Identity() Transform {
	return Transform{Rotation: Quaternion{W: 1}}
}

// Inverse 返回逆变换（父坐标系 -> 子坐标系）。
func (t Transform) Inverse() Transform {
	invRot := t.Rotation.conjugate()
	return Transform{
		Translation: invRot.rotate(t.Translation).neg(),
		Rotation:    invRot,
	}
}

// Compose 返回 a ∘ b：先施加 b 再施加 a。
func (a Transform) Compose(b Transform) Transform {
	return Transform{
		Translation: a.Translation.add(a.Rotation.rotate(b.Translation)),
		Rotation:    a.Rotation.mul(b.Rotation),
	}
}

// Normalize 返回单位化后的四元数。
func (q Quaternion) Normalize() Quaternion {
	norm := math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W)
	if norm == 0 {
		return Quaternion{W: 1}
	}
	return Quaternion{
		X: q.X / norm,
		Y: q.Y / norm,
		Z: q.Z / norm,
		W: q.W / norm,
	}
}

// interpolateTransform 在两帧采样之间按时间做线性插值（平移 LERP，旋转 SLERP）。
func interpolateTransform(a, b stampedTransform, at time.Time) Transform {
	span := b.ts.Sub(a.ts).Seconds()
	alpha := 0.0
	if span > 0 {
		alpha = (at.Sub(a.ts).Seconds()) / span
	}
	return Transform{
		Translation: a.tr.Translation.lerp(b.tr.Translation, alpha),
		Rotation:    a.tr.Rotation.slerp(b.tr.Rotation, alpha),
	}
}

func (v Vector3) add(o Vector3) Vector3 {
	return Vector3{X: v.X + o.X, Y: v.Y + o.Y, Z: v.Z + o.Z}
}

func (v Vector3) neg() Vector3 {
	return Vector3{X: -v.X, Y: -v.Y, Z: -v.Z}
}

func (v Vector3) scale(s float64) Vector3 {
	return Vector3{X: v.X * s, Y: v.Y * s, Z: v.Z * s}
}

func (v Vector3) lerp(o Vector3, alpha float64) Vector3 {
	return v.add(o.add(v.neg()).scale(alpha))
}

// conjugate 返回四元数共轭（单位四元数即逆）。
func (q Quaternion) conjugate() Quaternion {
	return Quaternion{X: -q.X, Y: -q.Y, Z: -q.Z, W: q.W}
}

// mul 返回四元数乘积 q * o。
func (q Quaternion) mul(o Quaternion) Quaternion {
	return Quaternion{
		X: q.W*o.X + q.X*o.W + q.Y*o.Z - q.Z*o.Y,
		Y: q.W*o.Y - q.X*o.Z + q.Y*o.W + q.Z*o.X,
		Z: q.W*o.Z + q.X*o.Y - q.Y*o.X + q.Z*o.W,
		W: q.W*o.W - q.X*o.X - q.Y*o.Y - q.Z*o.Z,
	}
}

// rotate 用单位四元数旋转一个向量。
func (q Quaternion) rotate(v Vector3) Vector3 {
	// q * v * q^{-1}，展开为：v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
	u := Vector3{X: q.X, Y: q.Y, Z: q.Z}
	c1 := u.cross(v).add(v.scale(q.W))
	c2 := u.cross(c1).scale(2)
	return v.add(c2)
}

func (v Vector3) cross(o Vector3) Vector3 {
	return Vector3{
		X: v.Y*o.Z - v.Z*o.Y,
		Y: v.Z*o.X - v.X*o.Z,
		Z: v.X*o.Y - v.Y*o.X,
	}
}

func (q Quaternion) dot(o Quaternion) float64 {
	return q.X*o.X + q.Y*o.Y + q.Z*o.Z + q.W*o.W
}

// slerp 在两条单位四元数之间做球面线性插值；夹角很小时退化为归一化 LERP。
func (q Quaternion) slerp(o Quaternion, alpha float64) Quaternion {
	q0 := q.Normalize()
	q1 := o.Normalize()
	dot := q0.dot(q1)
	if dot < 0 {
		q1 = q1.neg()
		dot = -dot
	}
	const threshold = 0.9995
	if dot > threshold {
		return Quaternion{
			X: q0.X + alpha*(q1.X-q0.X),
			Y: q0.Y + alpha*(q1.Y-q0.Y),
			Z: q0.Z + alpha*(q1.Z-q0.Z),
			W: q0.W + alpha*(q1.W-q0.W),
		}.Normalize()
	}
	theta0 := math.Acos(math.Min(1, math.Max(-1, dot)))
	theta := theta0 * alpha
	sinTheta0 := math.Sin(theta0)
	s0 := math.Cos(theta) - dot*math.Sin(theta)/sinTheta0
	s1 := math.Sin(theta) / sinTheta0
	return Quaternion{
		X: s0*q0.X + s1*q1.X,
		Y: s0*q0.Y + s1*q1.Y,
		Z: s0*q0.Z + s1*q1.Z,
		W: s0*q0.W + s1*q1.W,
	}.Normalize()
}

func (q Quaternion) neg() Quaternion {
	return Quaternion{X: -q.X, Y: -q.Y, Z: -q.Z, W: -q.W}
}

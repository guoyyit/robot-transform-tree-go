package tftree

import (
	"math"
	"time"
)

// Vector3 是三维平移向量（单位由发布方约定，服务不做解释）。
type Vector3 struct {
	X, Y, Z float64
}

// Quaternion 是单位四元数姿态，约定 W 为标量分量。
type Quaternion struct {
	X, Y, Z, W float64
}

// Transform 描述某坐标系相对其父坐标系的刚体位姿：
// 子坐标系原点在父坐标系中的位置，以及子->父的旋转。
type Transform struct {
	Translation Vector3
	Rotation    Quaternion
}

// TimestampedTransform 是带时间戳的一帧动态变换。
type TimestampedTransform struct {
	Timestamp time.Time
	Transform Transform
}

// identityTransform 返回单位变换。
func identityTransform() Transform {
	return Transform{Rotation: Quaternion{W: 1}}
}

// inverse 返回刚体变换的逆变换。
func (tr Transform) inverse() Transform {
	inv := quatConjugate(tr.Rotation)
	return Transform{
		Translation: quatRotate(inv, tr.Translation).scale(-1),
		Rotation:    inv,
	}
}

// compose 返回 tr 与 next 的复合：先应用 next 再应用 tr。
func (tr Transform) compose(next Transform) Transform {
	return Transform{
		Translation: tr.Translation.add(quatRotate(tr.Rotation, next.Translation)),
		Rotation:    quatMultiply(tr.Rotation, next.Rotation),
	}
}

// interpolate 在两帧之间做线性平移 + 球面线性插值（SLERP）。
// frac 为 0 时返回 a，为 1 时返回 b。
func interpolate(a, b TimestampedTransform, frac float64) Transform {
	if frac <= 0 {
		return a.Transform
	}
	if frac >= 1 {
		return b.Transform
	}
	return Transform{
		Translation: lerpVec(a.Transform.Translation, b.Transform.Translation, frac),
		Rotation:    slerp(a.Transform.Rotation, b.Transform.Rotation, frac),
	}
}

func (v Vector3) add(o Vector3) Vector3 {
	return Vector3{X: v.X + o.X, Y: v.Y + o.Y, Z: v.Z + o.Z}
}

func (v Vector3) scale(s float64) Vector3 {
	return Vector3{X: v.X * s, Y: v.Y * s, Z: v.Z * s}
}

func lerpVec(a, b Vector3, f float64) Vector3 {
	return a.add(b.add(a.scale(-1)).scale(f))
}

func quatConjugate(q Quaternion) Quaternion {
	return Quaternion{X: -q.X, Y: -q.Y, Z: -q.Z, W: q.W}
}

// quatMultiply 返回 Hamilton 乘积 qa * qb。
func quatMultiply(qa, qb Quaternion) Quaternion {
	return Quaternion{
		W: qa.W*qb.W - qa.X*qb.X - qa.Y*qb.Y - qa.Z*qb.Z,
		X: qa.W*qb.X + qa.X*qb.W + qa.Y*qb.Z - qa.Z*qb.Y,
		Y: qa.W*qb.Y - qa.X*qb.Z + qa.Y*qb.W + qa.Z*qb.X,
		Z: qa.W*qb.Z + qa.X*qb.Y - qa.Y*qb.X + qa.Z*qb.W,
	}
}

// quatRotate 用单位四元数 q 旋转向量 v（q v q*）。
func quatRotate(q Quaternion, v Vector3) Vector3 {
	// v' = v + 2 * q.xyz × (q.w * v + q.xyz × v)
	qv := Vector3{X: q.X, Y: q.Y, Z: q.Z}
	cross1 := v.scale(q.W).add(cross(qv, v))
	cross2 := cross(qv, cross1).scale(2)
	return v.add(cross2)
}

func cross(a, b Vector3) Vector3 {
	return Vector3{
		X: a.Y*b.Z - a.Z*b.Y,
		Y: a.Z*b.X - a.X*b.Z,
		Z: a.X*b.Y - a.Y*b.X,
	}
}

// slerp 在两个单位四元数之间做球面线性插值，自动选择短弧。
func slerp(a, b Quaternion, f float64) Quaternion {
	an, bn := normalizeQuat(a), normalizeQuat(b)
	dot := an.X*bn.X + an.Y*bn.Y + an.Z*bn.Z + an.W*bn.W
	if dot < 0 {
		bn = bn.scale(-1)
		dot = -dot
	}
	// 姿态几乎相同：退化为线性插值并归一化，避免除以零。
	if dot > 0.9995 {
		return normalizeQuat(Quaternion{
			X: an.X + f*(bn.X-an.X),
			Y: an.Y + f*(bn.Y-an.Y),
			Z: an.Z + f*(bn.Z-an.Z),
			W: an.W + f*(bn.W-an.W),
		})
	}
	theta0 := math.Acos(math.Max(-1, math.Min(1, dot)))
	theta := theta0 * f
	sinTheta0 := math.Sin(theta0)
	s1 := math.Cos(theta) - dot*math.Sin(theta)/sinTheta0
	s0 := math.Sin(theta) / sinTheta0
	return normalizeQuat(Quaternion{
		X: s1*an.X + s0*bn.X,
		Y: s1*an.Y + s0*bn.Y,
		Z: s1*an.Z + s0*bn.Z,
		W: s1*an.W + s0*bn.W,
	})
}

func (q Quaternion) scale(s float64) Quaternion {
	return Quaternion{X: q.X * s, Y: q.Y * s, Z: q.Z * s, W: q.W * s}
}

func normalizeQuat(q Quaternion) Quaternion {
	norm := math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W)
	if norm < 1e-12 {
		return Quaternion{W: 1}
	}
	return q.scale(1 / norm)
}

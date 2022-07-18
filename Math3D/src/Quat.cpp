#include "m3pch.h"
#include "Quat.h"
#include "Scalar.h"

namespace m3
{
	//static inline float clampf(float x, float min, float max)
	//{
	//	return x > max ? max : x < min ? min : x;
	//}

	Quat AngleAxis(const Vec3& axis, float angle, bool degree) {
		if (degree) angle = Deg2Rad(angle);
		Vec3 norm = Normalized(axis);
		float s = sinf(angle * 0.5f);

		return Quat(
			norm.x * s,
			norm.y * s,
			norm.z * s,
			cosf(angle * 0.5f)
		);
	}

	Quat FromTo(const Vec3& from, const Vec3& to) {
		Vec3 f = Normalized(from);
		Vec3 t = Normalized(to);

		if (f == t) {
			return Quat();
		}
		else if (f == t * -1.0f) {
			Vec3 ortho = Vec3(1, 0, 0);
			if (fabsf(f.y) < fabsf(f.x)) {
				ortho = Vec3(0, 1, 0);
			}
			if (fabsf(f.z) < fabs(f.y) && fabs(f.z) < fabsf(f.x)) {
				ortho = Vec3(0, 0, 1);
			}

			Vec3 axis = Normalized(Cross(f, ortho));
			return Quat(axis.x, axis.y, axis.z, 0);
		}

		Vec3 half = Normalized(f + t);
		Vec3 axis = Cross(f, half);

		return Quat(
			axis.x,
			axis.y,
			axis.z,
			Dot(f, half)
		);
	}

	Vec3 GetAxis(const Quat& Quat) {
		return Normalized(Vec3(Quat.x, Quat.y, Quat.z));
	}

	float GetAngle(const Quat& Quat, bool degree) {
		if (degree) return Rad2Deg(2.0f * acosf(Quat.w));
		else return 2.0f * acosf(Quat.w);
	}

	Quat operator+(const Quat& a, const Quat& b) {
		return Quat(
			a.x + b.x,
			a.y + b.y,
			a.z + b.z,
			a.w + b.w
		);
	}

	Quat operator-(const Quat& a, const Quat& b) {
		return Quat(
			a.x - b.x,
			a.y - b.y,
			a.z - b.z,
			a.w - b.w
		);
	}

	Quat operator*(const Quat& a, float b) {
		return Quat(
			a.x * b,
			a.y * b,
			a.z * b,
			a.w * b
		);
	}

	Quat operator-(const Quat& q) {
		return Quat(
			-q.x,
			-q.y,
			-q.z,
			-q.w
		);
	}

	bool operator==(const Quat& left, const Quat& right) {
		return (fabsf(left.x - right.x) <= QUAT_EPSILON && fabsf(left.y - right.y) <= QUAT_EPSILON && fabsf(left.z - right.z) <= QUAT_EPSILON && fabsf(left.w - left.w) <= QUAT_EPSILON);
	}

	bool operator!=(const Quat& a, const Quat& b) {
		return !(a == b);
	}

	std::ostream& operator<<(std::ostream& output,
		const Quat& q)
	{
		output << "Quat(" << q.x << "," << q.y << "," << q.z << "," << q.w << ")";
		return output;
	}

	bool SameOrientation(const Quat& left, const Quat& right) {
		return (fabsf(left.x - right.x) <= QUAT_EPSILON && fabsf(left.y - right.y) <= QUAT_EPSILON && fabsf(left.z - right.z) <= QUAT_EPSILON && fabsf(left.w - left.w) <= QUAT_EPSILON)
			|| (fabsf(left.x + right.x) <= QUAT_EPSILON && fabsf(left.y + right.y) <= QUAT_EPSILON && fabsf(left.z + right.z) <= QUAT_EPSILON && fabsf(left.w + left.w) <= QUAT_EPSILON);
	}

	float Dot(const Quat& a, const Quat& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	}

	float LenSq(const Quat& q) {
		return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	}

	float Len(const Quat& q) {
		float lenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		if (lenSq < QUAT_EPSILON) {
			return 0.0f;
		}
		return sqrtf(lenSq);
	}

	void Normalize(Quat& q) {
		float lenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		if (lenSq < QUAT_EPSILON) {
			return;
		}
		float i_len = 1.0f / sqrtf(lenSq);

		q.x *= i_len;
		q.y *= i_len;
		q.z *= i_len;
		q.w *= i_len;
	}

	Quat Normalized(const Quat& q) {
		float lenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		if (lenSq < QUAT_EPSILON) {
			return Quat();
		}
		float i_len = 1.0f / sqrtf(lenSq);

		return Quat(
			q.x * i_len,
			q.y * i_len,
			q.z * i_len,
			q.w * i_len
		);
	}

	Quat Conjugate(const Quat& q) {
		return Quat(
			-q.x,
			-q.y,
			-q.z,
			q.w
		);
	}

	Quat Inverse(const Quat& q) {
		float lenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		if (lenSq < QUAT_EPSILON) {
			return Quat();
		}
		float recip = 1.0f / lenSq;

		// conjugate / norm
		return Quat(
			-q.x * recip,
			-q.y * recip,
			-q.z * recip,
			q.w * recip
		);
	}
	Quat operator*(const Quat& Q1, const Quat& Q2) {  // NOTE: I changed the order for compute
		return Quat(
			Q1.x * Q2.w + Q1.y * Q2.z - Q1.z * Q2.y + Q1.w * Q2.x,
			-Q1.x * Q2.z + Q1.y * Q2.w + Q1.z * Q2.x + Q1.w * Q2.y,
			Q1.x * Q2.y - Q1.y * Q2.x + Q1.z * Q2.w + Q1.w * Q2.z,
			-Q1.x * Q2.x - Q1.y * Q2.y - Q1.z * Q2.z + Q1.w * Q2.w
		);
	}

	Vec3 operator*(const Quat& q, const Vec3& v) {
		return    q.vector * 2.0f * Dot(q.vector, v) +
			v * (q.scalar * q.scalar - Dot(q.vector, q.vector)) +
			Cross(q.vector, v) * 2.0f * q.scalar;
	}

	Quat Mix(const Quat& from, const Quat& to, float t) {
		return from * (1.0f - t) + to * t;
	}

	Quat Nlerp(const Quat& from, const Quat& to, float t) {
		return Normalized(from + (to - from) * t);
	}

	Quat operator^(const Quat& q, float f) {
		float angle = 2.0f * acosf(q.scalar);
		Vec3 axis = Normalized(q.vector);

		float halfCos = cosf(f * angle * 0.5f);
		float halfSin = sinf(f * angle * 0.5f);

		return Quat(
			axis.x * halfSin,
			axis.y * halfSin,
			axis.z * halfSin,
			halfCos
		);
	}

	Quat Slerp(const Quat& start, const Quat& end, float t) {
		if (fabsf(Dot(start, end)) > 1.0f - QUAT_EPSILON) {
			return Nlerp(start, end, t);
		}

		return Normalized(((Inverse(start) * end) ^ t) * start);
	}

	Quat LookRotation(const Vec3& direcion, const Vec3& up) {
		// Find orthonormal basis vectors
		Vec3 f = Normalized(direcion);
		Vec3 u = Normalized(up);
		Vec3 r = Cross(u, f);
		u = Cross(f, r);

		// From world forward to object forward
		Quat f2d = FromTo(Vec3(0, 0, 1), f);

		// what direction is the new object up?
		Vec3 objectUp = f2d * Vec3(0, 1, 0);
		// From object up to desired up
		Quat u2u = FromTo(objectUp, u);

		// Rotate to forward direction first, then twist to correct up
		Quat result = f2d * u2u;  // TODO: Need to check out. Order changed.
		// Don't forget to normalize the result
		return Normalized(result);
	}

	Mat4 QuatToMat4(const Quat& q) {
		Vec3 r = q * Vec3(1, 0, 0);
		Vec3 u = q * Vec3(0, 1, 0);
		Vec3 f = q * Vec3(0, 0, 1);

		return Mat4(
			r.x, r.y, r.z, 0,
			u.x, u.y, u.z, 0,
			f.x, f.y, f.z, 0,
			0, 0, 0, 1
		);
	}

	Quat Mat4ToQuat(const Mat4& m) {
		Vec3 up = Normalized(Vec3(m.up.x, m.up.y, m.up.z));
		Vec3 forward = Normalized(Vec3(m.forward.x, m.forward.y, m.forward.z));
		Vec3 right = Cross(up, forward);
		up = Cross(forward, right);

		return LookRotation(forward, up);
	}

	// TODO: is it correct?
	Quat QuatFromEulerXYZ(float x, float y, float z)
	{
		return AngleAxis(Vec3(0, 0, 1), z) * AngleAxis(Vec3(0, 1, 0), y) * AngleAxis(Vec3(1, 0, 0), x);
	}

	Quat QuatFromEulerZYX(float x, float y, float z)
	{
		return AngleAxis(Vec3(0, 0, 1), x) * AngleAxis(Vec3(0, 1, 0), y) * AngleAxis(Vec3(1, 0, 0), z);
	}

	Vec3 QuatLog(Quat q, float eps)
	{
		float length = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
		if (length < eps)
		{
			return Vec3(q.x, q.y, q.z);
		}
		else
		{
			float halfangle = acosf(Clamp(q.w, -1.0f, 1.0f));
			return halfangle * (Vec3(q.x, q.y, q.z) / length);
		}
	}

	Vec3 QuatToScaledAngleAxis(Quat q, float eps)
	{
		return 2.0f * QuatLog(q, eps);
	}

	Quat QuatInvMul(Quat q, Quat p)
	{
		return QuatMul(QuatInv(q), p);
	}

	Quat QuatInv(Quat q)
	{
		return Quat(q.x, q.y, q.z, -q.w);
	}

	Quat QuatMul(Quat q, Quat p)
	{
		return Quat(
			p.w * q.x + p.x * q.w - p.y * q.z + p.z * q.y,
			p.w * q.y + p.x * q.z + p.y * q.w - p.z * q.x,
			p.w * q.z - p.x * q.y + p.y * q.x + p.z * q.w,
			p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z);
	}

	Quat QuatFromAngleAxis(float angle, Vec3 axis)
	{
		float c = cosf(angle / 2.0f);
		float s = sinf(angle / 2.0f);
		return Quat(s * axis.x, s * axis.y, s * axis.z, c);
	}

	Quat QuatSlerpShortestApprox(Quat q, Quat p, float alpha)
	{
		float ca = Dot(q, p);

		if (ca < 0.0f)
		{
			p = -p;
		}

		float d = fabsf(ca);
		float a = 1.0904f + d * (-3.2452f + d * (3.55645f - d * 1.43519f));
		float b = 0.848013f + d * (-1.06021f + d * 0.215638f);
		float k = a * (alpha - 0.5f) * (alpha - 0.5f) + b;
		float oalpha = alpha + alpha * (alpha - 0.5f) * (alpha - 1) * k;

		return Nlerp(q, p, oalpha);
	}

	Quat QuatAbs(Quat x)
	{
		return x.w < 0.0 ? -x : x;
	}

	Quat QuatFromScaledAngleAxis(Vec3 v, float eps)
	{
		return QuatExp(v / 2.0f, eps);
	}

	Quat QuatExp(Vec3 v, float eps)
	{
		float halfangle = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

		if (halfangle < eps)
		{
			return Normalized(Quat(v.x, v.y, v.z, 1.0f));
		}
		else
		{
			float c = cosf(halfangle);
			float s = sinf(halfangle) / halfangle;
			return Quat(s * v.x, s * v.y, s * v.z, c);
		}
	}

	Vec3 ToEuler(const Quat& q, const std::string& order, bool degree)
	{
		Mat4 m = QuatToMat4(q);
		return ToEuler(m, order, degree);
	}

}

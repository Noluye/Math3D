#include "m3pch.h"
#include "Vec3.h"
#include "Scalar.h"
#include "Quat.h"

namespace m3
{
	Vec3 operator+(const Vec3& l, const Vec3& r) {
		return Vec3(l.x + r.x, l.y + r.y, l.z + r.z);
	}

	Vec3 operator-(const Vec3& l, const Vec3& r) {
		return Vec3(l.x - r.x, l.y - r.y, l.z - r.z);
	}

	Vec3 operator-(Vec3 v)
	{
		return Vec3(-v.x, -v.y, -v.z);
	}

	Vec3 operator*(const Vec3& v, float f) {
		return Vec3(v.x * f, v.y * f, v.z * f);
	}

	Vec3 operator*(float s, Vec3 v)
	{
		return Vec3(v.x * s, v.y * s, v.z * s);
	}

	Vec3 operator*(const Vec3& l, const Vec3& r) {
		return Vec3(l.x * r.x, l.y * r.y, l.z * r.z);
	}

	Vec3 operator/(Vec3 v, float s)
	{
		return Vec3(v.x / s, v.y / s, v.z / s);
	}

	Vec3 operator/(float s, Vec3 v)
	{
		return Vec3(s / v.x, s / v.y, s / v.z);
	}
	Vec3 operator/(Vec3 v, Vec3 w)
	{
		return Vec3(v.x / w.x, v.y / w.y, v.z / w.z);
	}
	std::ostream& operator<<(std::ostream& output,
		const Vec3& v)
	{
		output << "Vec3(" << v.x << "," << v.y << "," << v.z << ")";
		return output;
	}

	float Dot(const Vec3& l, const Vec3& r) {
		return l.x * r.x + l.y * r.y + l.z * r.z;
	}

	float LenSq(const Vec3& v) {
		return v.x * v.x + v.y * v.y + v.z * v.z;
	}

	float Len(const Vec3& v) {
		float lenSq = v.x * v.x + v.y * v.y + v.z * v.z;
		if (lenSq < VEC3_EPSILON) {
			return 0.0f;
		}
		return sqrtf(lenSq);
	}

	void Normalize(Vec3& v) {
		float lenSq = v.x * v.x + v.y * v.y + v.z * v.z;
		if (lenSq < VEC3_EPSILON) {
			return;
		}
		float invLen = 1.0f / sqrtf(lenSq);

		v.x *= invLen;
		v.y *= invLen;
		v.z *= invLen;
	}

	Vec3 Normalized(const Vec3& v) {
		float lenSq = v.x * v.x + v.y * v.y + v.z * v.z;
		if (lenSq < VEC3_EPSILON) {
			return v;
		}
		float invLen = 1.0f / sqrtf(lenSq);

		return Vec3(
			v.x * invLen,
			v.y * invLen,
			v.z * invLen
		);
	}

	float Angle(const Vec3& l, const Vec3& r) {
		float sqMagL = l.x * l.x + l.y * l.y + l.z * l.z;
		float sqMagR = r.x * r.x + r.y * r.y + r.z * r.z;

		if (sqMagL < VEC3_EPSILON || sqMagR < VEC3_EPSILON) {
			return 0.0f;
		}

		float dot = l.x * r.x + l.y * r.y + l.z * r.z;
		float len = sqrtf(sqMagL) * sqrtf(sqMagR);
		return acosf(dot / len);
	}

	Vec3 Project(const Vec3& a, const Vec3& b) {
		float magBSq = Len(b);
		if (magBSq < VEC3_EPSILON) {
			return Vec3();
		}
		float scale = Dot(a, b) / magBSq;
		return b * scale;
	}

	Vec3 Reject(const Vec3& a, const Vec3& b) {
		Vec3 projection = Project(a, b);
		return a - projection;
	}

	Vec3 Reflect(const Vec3& a, const Vec3& b) {
		float magBSq = Len(b);
		if (magBSq < VEC3_EPSILON) {
			return Vec3();
		}
		float scale = Dot(a, b) / magBSq;
		Vec3 proj2 = b * (scale * 2);
		return a - proj2;
	}

	Vec3 Cross(const Vec3& l, const Vec3& r) {
		return Vec3(
			l.y * r.z - l.z * r.y,
			l.z * r.x - l.x * r.z,
			l.x * r.y - l.y * r.x
		);
	}

	Vec3 Lerp(const Vec3& s, const Vec3& e, float t) {
		return Vec3(
			s.x + (e.x - s.x) * t,
			s.y + (e.y - s.y) * t,
			s.z + (e.z - s.z) * t
		);
	}

	Vec3 Slerp(const Vec3& s, const Vec3& e, float t) {
		if (t < 0.01f) {
			return Lerp(s, e, t);
		}

		Vec3 from = Normalized(s);
		Vec3 to = Normalized(e);

		float theta = Angle(from, to);
		float sin_theta = sinf(theta);

		float a = sinf((1.0f - t) * theta) / sin_theta;
		float b = sinf(t * theta) / sin_theta;

		return from * a + to * b;
	}

	Vec3 Nlerp(const Vec3& s, const Vec3& e, float t) {
		Vec3 linear(
			s.x + (e.x - s.x) * t,
			s.y + (e.y - s.y) * t,
			s.z + (e.z - s.z) * t
		);
		return Normalized(linear);
	}

	bool operator==(const Vec3& l, const Vec3& r) {
		Vec3 diff(l - r);
		return LenSq(diff) < VEC3_EPSILON;
	}

	bool operator!=(const Vec3& l, const Vec3& r) {
		return !(l == r);
	}

	Vec3 Min(Vec3 v, Vec3 w)
	{
		return Vec3(Min(v.x, w.x), Min(v.y, w.y), Min(v.z, w.z));
	}

	Vec3 Max(Vec3 v, Vec3 w)
	{
		return Vec3(Max(v.x, w.x), Max(v.y, w.y), Max(v.z, w.z));
	}

	Vec3 Clamp(Vec3 v, Vec3 min, Vec3 max)
	{
		return Vec3(
			Clamp(v.x, min.x, max.x),
			Clamp(v.y, min.y, max.y),
			Clamp(v.z, min.z, max.z));
	}

	void OrthoNormalize(Vec3& normal, Vec3& tangent)
	{
		Normalize(normal);
		tangent = tangent - Project(tangent, normal);
		Normalize(tangent);
	}

	Vec3 Rad2Deg(Vec3 rad) { return rad * 180 / PI; }

	Vec3 Deg2Rad(Vec3 deg) { return deg * PI / 180; }
}

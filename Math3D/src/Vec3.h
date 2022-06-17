#pragma once
#include <iostream>

namespace m3
{
#define VEC3_EPSILON 0.000001f

	struct Vec3 {
		union {
			struct {
				float x;
				float y;
				float z;
			};
			float v[3];
		};
		inline Vec3() : x(0.0f), y(0.0f), z(0.0f) { }
		inline Vec3(float _x, float _y, float _z) :
			x(_x), y(_y), z(_z) { }
		inline Vec3(float* fv) :
			x(fv[0]), y(fv[1]), z(fv[2]) { }

		friend std::ostream& operator<<(std::ostream& output, const Vec3& v);

		inline void Add_(const Vec3& rhs)
		{
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
		}

		inline static Vec3 Left()    { return Vec3{ 1.0f, 0.0f, 0.0f }; };
		inline static Vec3 Up()      { return Vec3{ 0.0f, 1.0f, 0.0f }; };
		inline static Vec3 Forward() { return Vec3{ 0.0f, 0.0f, 1.0f }; };
	};

	Vec3 operator+(const Vec3& l, const Vec3& r);
	Vec3 operator-(const Vec3& l, const Vec3& r);
	Vec3 operator-(Vec3 v);
	// TODO: operator+= / operator-=
	Vec3 operator*(const Vec3& v, float s);
	Vec3 operator*(float s, Vec3 v);
	Vec3 operator*(const Vec3& l, const Vec3& r);
	Vec3 operator/(Vec3 v, float s);
	Vec3 operator/(float s, Vec3 v);
	Vec3 operator/(Vec3 v, Vec3 w);
	float Dot(const Vec3& l, const Vec3& r);
	float LenSq(const Vec3& v);
	float Len(const Vec3& v);
	void Normalize(Vec3& v);
	Vec3 Normalized(const Vec3& v);
	float Angle(const Vec3& l, const Vec3& r);
	Vec3 Project(const Vec3& a, const Vec3& b);
	Vec3 Reject(const Vec3& a, const Vec3& b);
	Vec3 Reflect(const Vec3& a, const Vec3& b);
	Vec3 Cross(const Vec3& l, const Vec3& r);
	Vec3 Lerp(const Vec3& s, const Vec3& e, float t);
	Vec3 Slerp(const Vec3& s, const Vec3& e, float t);
	Vec3 Nlerp(const Vec3& s, const Vec3& e, float t);
	bool operator==(const Vec3& l, const Vec3& r);
	bool operator!=(const Vec3& l, const Vec3& r);
	Vec3 Min(Vec3 v, Vec3 w);
	Vec3 Max(Vec3 v, Vec3 w);
	Vec3 Clamp(Vec3 v, Vec3 min, Vec3 max);
	/// <summary>
	/// Makes vectors normalized and orthogonal to each other.
	/// Normalizes normal. Normalizes tangent and makes sure it is orthogonal to normal (that is, angle between them is 90 degrees).
	/// </summary>
	/// <param name="v1"></param>
	/// <param name="v2"></param>
	void OrthoNormalize(Vec3& normal, Vec3& tangent);
}

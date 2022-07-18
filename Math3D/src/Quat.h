#pragma once

#include "Vec3.h"
#include "Mat4.h"

namespace m3
{
#define QUAT_EPSILON 0.000001f

	struct Quat {
		union {
			struct {
				float x;
				float y;
				float z;
				float w;
			};
			struct {
				Vec3 vector;
				float scalar;
			};
			float v[4];
		};

		inline Quat() :
			x(0), y(0), z(0), w(1) { }
		inline Quat(float _x, float _y, float _z, float _w) :
			x(_x), y(_y), z(_z), w(_w) {}
		friend std::ostream& operator<<(std::ostream& output, const Quat& q);
	};

	Quat AngleAxis(const Vec3& axis, float angle, bool degree = false);
	Quat FromTo(const Vec3& from, const Vec3& to);
	Vec3 GetAxis(const Quat& Quat);
	float GetAngle(const Quat& Quat, bool degree = false);
	Quat operator+(const Quat& a, const Quat& b);
	Quat operator-(const Quat& a, const Quat& b);
	Quat operator*(const Quat& a, float b);
	Quat operator-(const Quat& q);
	bool operator==(const Quat& left, const Quat& right);
	bool operator!=(const Quat& a, const Quat& b);
	bool SameOrientation(const Quat& left, const Quat& right);
	float Dot(const Quat& a, const Quat& b);
	float LenSq(const Quat& q);
	float Len(const Quat& q);
	void Normalize(Quat& q);
	Quat Normalized(const Quat& q);
	Quat Conjugate(const Quat& q);
	Quat Inverse(const Quat& q);
	Quat operator*(const Quat& Q1, const Quat& Q2);
	Vec3 operator*(const Quat& q, const Vec3& v);
	Quat Mix(const Quat& from, const Quat& to, float t);
	Quat Nlerp(const Quat& from, const Quat& to, float t);
	Quat operator^(const Quat& q, float f);
	Quat Slerp(const Quat& start, const Quat& end, float t);
	Quat LookRotation(const Vec3& direcion, const Vec3& up);
	Mat4 QuatToMat4(const Quat& q);
	Quat Mat4ToQuat(const Mat4& m);

	Quat QuatFromEulerXYZ(float x, float y, float z);
	Quat QuatFromEulerZYX(float x, float y, float z);
	Vec3 QuatLog(Quat q, float eps = QUAT_EPSILON);
	/// <summary>
	/// deprecated
	/// </summary>
	Vec3 QuatToScaledAngleAxis(Quat q, float eps = QUAT_EPSILON);
	Quat QuatInvMul(Quat q, Quat p);
	Quat QuatInv(Quat q);
	Quat QuatMul(Quat q, Quat p);
	Quat QuatFromAngleAxis(float angle, Vec3 axis);
	Quat QuatSlerpShortestApprox(Quat q, Quat p, float alpha);  // Taken from https://zeux.io/2015/07/23/approximating-slerp/
	Quat QuatAbs(Quat x);
	Quat QuatFromScaledAngleAxis(Vec3 v, float eps = 1e-8f);
	Quat QuatExp(Vec3 v, float eps = 1e-8f);

	Vec3 ToEuler(const Quat& q, const std::string& order = "XYZ", bool degree = true);
}

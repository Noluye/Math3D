#pragma once

#include "Vec3.h"
#include "Mat4.h"
#include "Quat.h"

namespace m3
{
	struct Transform {
		Vec3 position;
		Quat rotation;
		Vec3 scale;
		Transform() :
			position(Vec3(0.0)),
			rotation(Quat(0, 0, 0, 1)),
			scale(Vec3(1.0)) {}
		Transform(const Vec3& p, const Quat& r) :
			position(p), rotation(r), scale(Vec3(1.0)) {}
		Transform(const Vec3& p, const Quat& r, const Vec3& s) :
			position(p), rotation(r), scale(s) {}
	}; // End of transform struct
	
	Transform Combine(const Transform& parent, const Transform& curr);
	Transform operator*(const Transform& parent, const Transform& curr);
	Transform Inverse(const Transform& t);
	Transform Mix(const Transform& a, const Transform& b, float t);
	bool operator==(const Transform& a, const Transform& b);
	bool operator!=(const Transform& a, const Transform& b);
	Mat4 TransformToMat4(const Transform& t);
	Transform Mat4ToTransform(const Mat4& m);
	Vec3 TransformPoint(const Transform& a, const Vec3& b);
	Vec3 TransformVector(const Transform& a, const Vec3& b);
}


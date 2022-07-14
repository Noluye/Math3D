#include "m3pch.h"
#include "Transform.h"


namespace m3
{
	Transform Combine(const Transform& a, const Transform& b) {
		Transform out;

		out.scale = a.scale * b.scale;
		out.rotation = b.rotation * a.rotation;

		out.position = a.rotation * (a.scale * b.position);
		out.position = a.position + out.position;

		return out;
	}

	Transform Inverse(const Transform& t) {
		Transform inv;

		inv.rotation = Inverse(t.rotation);

		inv.scale.x = fabs(t.scale.x) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.x;
		inv.scale.y = fabs(t.scale.y) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.y;
		inv.scale.z = fabs(t.scale.z) < VEC3_EPSILON ? 0.0f : 1.0f / t.scale.z;

		Vec3 invTranslation = t.position * -1.0f;
		inv.position = inv.rotation * (inv.scale * invTranslation);

		return inv;
	}

	Transform Mix(const Transform& a, const Transform& b, float t) {
		Quat bRot = b.rotation;
		if (Dot(a.rotation, bRot) < 0.0f) {
			bRot = -bRot;
		}
		return Transform(
			Lerp(a.position, b.position, t),
			Nlerp(a.rotation, bRot, t),
			Lerp(a.scale, b.scale, t));
	}

	bool operator==(const Transform& a, const Transform& b) {
		return a.position == b.position &&
			a.rotation == b.rotation &&
			a.scale == b.scale;
	}

	bool operator!=(const Transform& a, const Transform& b) {
		return !(a == b);
	}

	Mat4 TransformToMat4(const Transform& t) {
		// First, extract the rotation basis of the transform
		Vec3 x = t.rotation * Vec3(1, 0, 0);
		Vec3 y = t.rotation * Vec3(0, 1, 0);
		Vec3 z = t.rotation * Vec3(0, 0, 1);

		// Next, scale the basis vectors
		x = x * t.scale.x;
		y = y * t.scale.y;
		z = z * t.scale.z;

		// Extract the position of the transform
		Vec3 p = t.position;

		// Create matrix
		return Mat4(
			x.x, x.y, x.z, 0, // X basis (& Scale)
			y.x, y.y, y.z, 0, // Y basis (& scale)
			z.x, z.y, z.z, 0, // Z basis (& scale)
			p.x, p.y, p.z, 1  // Position
		);
	}

	Transform Mat4ToTransform(const Mat4& m) {
		Transform out;

		out.position = Vec3(m.v[12], m.v[13], m.v[14]);
		out.rotation = Mat4ToQuat(m);

		Mat4 rotScaleMat(
			m.v[0], m.v[1], m.v[2], 0,
			m.v[4], m.v[5], m.v[6], 0,
			m.v[8], m.v[9], m.v[10], 0,
			0, 0, 0, 1
		);
		Mat4 invRotMat = QuatToMat4(Inverse(out.rotation));
		Mat4 scaleSkewMat = rotScaleMat * invRotMat;

		out.scale = Vec3(
			scaleSkewMat.v[0],
			scaleSkewMat.v[5],
			scaleSkewMat.v[10]
		);

		return out;
	}

	Vec3 TransformPoint(const Transform& a, const Vec3& b) {
		Vec3 out;

		out = a.rotation * (a.scale * b);
		out = a.position + out;

		return out;
	}

	Vec3 TransformVector(const Transform& a, const Vec3& b) {
		Vec3 out;

		out = a.rotation * (a.scale * b);

		return out;
	}
}


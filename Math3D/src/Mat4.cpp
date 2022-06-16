#include "m3pch.h"
#include "Mat4.h"
#include "Scalar.h"

namespace m3
{
	bool operator==(const Mat4& a, const Mat4& b) {
		for (int i = 0; i < 16; ++i) {
			if (fabsf(a.v[i] - b.v[i]) > MAT4_EPSILON) {
				return false;
			}
		}
		return true;
	}

	bool operator!=(const Mat4& a, const Mat4& b) {
		return !(a == b);
	}

	Mat4 operator*(const Mat4& m, float f) {
		return Mat4(
			m.xx * f, m.xy * f, m.xz * f, m.xw * f,
			m.yx * f, m.yy * f, m.yz * f, m.yw * f,
			m.zx * f, m.zy * f, m.zz * f, m.zw * f,
			m.tx * f, m.ty * f, m.tz * f, m.tw * f
		);
	}

	Mat4 operator+(const Mat4& a, const Mat4& b) {
		return Mat4(
			a.xx + b.xx, a.xy + b.xy, a.xz + b.xz, a.xw + b.xw,
			a.yx + b.yx, a.yy + b.yy, a.yz + b.yz, a.yw + b.yw,
			a.zx + b.zx, a.zy + b.zy, a.zz + b.zz, a.zw + b.zw,
			a.tx + b.tx, a.ty + b.ty, a.tz + b.tz, a.tw + b.tw
		);
	}

#define M4D(aRow, bCol) \
a.v[0 * 4 + aRow] * b.v[bCol * 4 + 0] + \
a.v[1 * 4 + aRow] * b.v[bCol * 4 + 1] + \
a.v[2 * 4 + aRow] * b.v[bCol * 4 + 2] + \
a.v[3 * 4 + aRow] * b.v[bCol * 4 + 3]

	Mat4 operator*(const Mat4& a, const Mat4& b) {
		return Mat4(
			M4D(0, 0), M4D(1, 0), M4D(2, 0), M4D(3, 0), // Column 0
			M4D(0, 1), M4D(1, 1), M4D(2, 1), M4D(3, 1), // Column 1
			M4D(0, 2), M4D(1, 2), M4D(2, 2), M4D(3, 2), // Column 2
			M4D(0, 3), M4D(1, 3), M4D(2, 3), M4D(3, 3)  // Column 3
		);
	}

#define M4V4D(mRow, x, y, z, w) \
x * m.v[0 * 4 + mRow] + \
y * m.v[1 * 4 + mRow] + \
z * m.v[2 * 4 + mRow] + \
w * m.v[3 * 4 + mRow]

	Vec4 operator*(const Mat4& m, const Vec4& v) {
		return Vec4(
			M4V4D(0, v.x, v.y, v.z, v.w),
			M4V4D(1, v.x, v.y, v.z, v.w),
			M4V4D(2, v.x, v.y, v.z, v.w),
			M4V4D(3, v.x, v.y, v.z, v.w)
		);
	}

	Vec3 transformVector(const Mat4& m, const Vec3& v) {
		return Vec3(
			M4V4D(0, v.x, v.y, v.z, 0.0f),
			M4V4D(1, v.x, v.y, v.z, 0.0f),
			M4V4D(2, v.x, v.y, v.z, 0.0f)
		);
	}

	Vec3 transformPoint(const Mat4& m, const Vec3& v) {
		return Vec3(
			M4V4D(0, v.x, v.y, v.z, 1.0f),
			M4V4D(1, v.x, v.y, v.z, 1.0f),
			M4V4D(2, v.x, v.y, v.z, 1.0f)
		);
	}

	Vec3 transformPoint(const Mat4& m, const Vec3& v, float& w) {
		float _w = w;
		w = M4V4D(3, v.x, v.y, v.z, _w);
		return Vec3(
			M4V4D(0, v.x, v.y, v.z, _w),
			M4V4D(1, v.x, v.y, v.z, _w),
			M4V4D(2, v.x, v.y, v.z, _w)
		);
	}

#define M4SWAP(x, y) \
{float t = x; x = y; y = t; }

	void transpose(Mat4& m) {
		M4SWAP(m.yx, m.xy);
		M4SWAP(m.zx, m.xz);
		M4SWAP(m.tx, m.xw);
		M4SWAP(m.zy, m.yz);
		M4SWAP(m.ty, m.yw);
		M4SWAP(m.tz, m.zw);
	}

	Mat4 transposed(const Mat4& m) {
		return Mat4(
			m.xx, m.yx, m.zx, m.tx,
			m.xy, m.yy, m.zy, m.ty,
			m.xz, m.yz, m.zz, m.tz,
			m.xw, m.yw, m.zw, m.tw
		);
	}

#define M4_3X3MINOR(c0, c1, c2, r0, r1, r2) \
(m.v[c0 * 4 + r0] * (m.v[c1 * 4 + r1] * m.v[c2 * 4 + r2] - m.v[c1 * 4 + r2] * m.v[c2 * 4 + r1]) - \
    m.v[c1 * 4 + r0] * (m.v[c0 * 4 + r1] * m.v[c2 * 4 + r2] - m.v[c0 * 4 + r2] * m.v[c2 * 4 + r1]) + \
    m.v[c2 * 4 + r0] * (m.v[c0 * 4 + r1] * m.v[c1 * 4 + r2] - m.v[c0 * 4 + r2] * m.v[c1 * 4 + r1]))

	float determinant(const Mat4& m) {
		return  m.v[0] * M4_3X3MINOR(1, 2, 3, 1, 2, 3)
			- m.v[4] * M4_3X3MINOR(0, 2, 3, 1, 2, 3)
			+ m.v[8] * M4_3X3MINOR(0, 1, 3, 1, 2, 3)
			- m.v[12] * M4_3X3MINOR(0, 1, 2, 1, 2, 3);
	}

	Mat4 adjugate(const Mat4& m) {
		// Cofactor(M[i, j]) = Minor(M[i, j]] * pow(-1, i + j)
		Mat4 cofactor;

		cofactor.v[0] = M4_3X3MINOR(1, 2, 3, 1, 2, 3);
		cofactor.v[1] = -M4_3X3MINOR(1, 2, 3, 0, 2, 3);
		cofactor.v[2] = M4_3X3MINOR(1, 2, 3, 0, 1, 3);
		cofactor.v[3] = -M4_3X3MINOR(1, 2, 3, 0, 1, 2);

		cofactor.v[4] = -M4_3X3MINOR(0, 2, 3, 1, 2, 3);
		cofactor.v[5] = M4_3X3MINOR(0, 2, 3, 0, 2, 3);
		cofactor.v[6] = -M4_3X3MINOR(0, 2, 3, 0, 1, 3);
		cofactor.v[7] = M4_3X3MINOR(0, 2, 3, 0, 1, 2);

		cofactor.v[8] = M4_3X3MINOR(0, 1, 3, 1, 2, 3);
		cofactor.v[9] = -M4_3X3MINOR(0, 1, 3, 0, 2, 3);
		cofactor.v[10] = M4_3X3MINOR(0, 1, 3, 0, 1, 3);
		cofactor.v[11] = -M4_3X3MINOR(0, 1, 3, 0, 1, 2);

		cofactor.v[12] = -M4_3X3MINOR(0, 1, 2, 1, 2, 3);
		cofactor.v[13] = M4_3X3MINOR(0, 1, 2, 0, 2, 3);
		cofactor.v[14] = -M4_3X3MINOR(0, 1, 2, 0, 1, 3);
		cofactor.v[15] = M4_3X3MINOR(0, 1, 2, 0, 1, 2);

		return transposed(cofactor);
	}

	std::ostream& operator<<(std::ostream& output, const Mat4& m)
	{
		output << "Mat4(" << m.xx << "," << m.xy << "," << m.xz << "," << m.xw << std::endl
			<< m.yx << "," << m.yy << "," << m.yz << "," << m.yw << std::endl
			<< m.zx << "," << m.zy << "," << m.zz << "," << m.zw << std::endl
			<< m.tx << "," << m.ty << "," << m.tz << "," << m.tw << std::endl
			<< ")";
		return output;
	}

	Mat4 inverse(const Mat4& m) {
		float det = determinant(m);

		if (det == 0.0f) { // Epsilon check would need to be REALLY small
			std::cout << "WARNING: Trying to invert a matrix with a zero determinant\n";
			return Mat4();
		}
		Mat4 adj = adjugate(m);

		return adj * (1.0f / det);
	}

	void invert(Mat4& m) {
		float det = determinant(m);

		if (det == 0.0f) {
			std::cout << "WARNING: Trying to invert a matrix with a zero determinant\n";
			m = Mat4();
			return;
		}

		m = adjugate(m) * (1.0f / det);
	}

	Mat4 frustum(float l, float r, float b, float t, float n, float f) {
		if (l == r || t == b || n == f) {
			std::cout << "WARNING: Trying to create invalid frustum\n";
			return Mat4(); // Error
		}
		return Mat4(
			(2.0f * n) / (r - l), 0, 0, 0,
			0, (2.0f * n) / (t - b), 0, 0,
			(r + l) / (r - l), (t + b) / (t - b), (-(f + n)) / (f - n), -1,
			0, 0, (-2 * f * n) / (f - n), 0
		);
	}

	Mat4 perspective(float fov, float aspect, float znear, float zfar) {
		float ymax = znear * tanf(fov * 3.14159265359f / 360.0f);
		float xmax = ymax * aspect;

		return frustum(-xmax, xmax, -ymax, ymax, znear, zfar);
	}

	Mat4 ortho(float l, float r, float b, float t, float n, float f) {
		if (l == r || t == b || n == f) {
			return Mat4(); // Error
		}
		return Mat4(
			2.0f / (r - l), 0, 0, 0,
			0, 2.0f / (t - b), 0, 0,
			0, 0, -2.0f / (f - n), 0,
			-((r + l) / (r - l)), -((t + b) / (t - b)), -((f + n) / (f - n)), 1
		);
	}

	Mat4 lookAt(const Vec3& position, const Vec3& target, const Vec3& up) {
		// Remember, forward is negative z
		Vec3 f = Normalized(target - position) * -1.0f;
		Vec3 r = Cross(up, f); // Right handed
		if (r == Vec3(0, 0, 0)) {
			return Mat4(); // Error
		}
		Normalize(r);
		Vec3 u = Normalized(Cross(f, r)); // Right handed

		Vec3 t = Vec3(
			-Dot(r, position),
			-Dot(u, position),
			-Dot(f, position)
		);

		return Mat4(
			// Transpose upper 3x3 matrix to invert it
			r.x, u.x, f.x, 0,
			r.y, u.y, f.y, 0,
			r.z, u.z, f.z, 0,
			t.x, t.y, t.z, 1
		);
	}

	Vec3 to_euler(const Mat4& m, const std::string& order)
	{
		double a = m.xx;
		double f = m.yx;
		float g = m.zx;
		double h = m.xy;
		double k = m.yy;
		float l = m.zy;
		double s = m.xz;
		double n = m.yz;
		double e = m.zz;

		Vec3 euler = {};
		float& x = euler.x;
		float& y = euler.y;
		float& z = euler.z;

		if ("XYZ" == order) {
			y = std::asin(Clamp(g, -1, 1));

			if (0.999999 > std::abs(g)) {
				x = std::atan2(-l, e);
				z = std::atan2(-f, a);
			}
			else
			{
				x = std::atan2(n, k);
				z = 0;
			}
		}
		else if ("YXZ" == order) {
			x = std::asin(-Clamp(l, -1, 1));

			if (0.999999 > std::abs(l)) {
				y = std::atan2(g, e);
				z = std::atan2(h, k);
			}
			else
			{
				y = std::atan2(-s, a);
				z = 0;
			}
		}
		else if ("ZXY" == order) {
			x = std::asin(Clamp(n, -1, 1));

			if (0.999999 > std::abs(n)) {
				y = std::atan2(-s, e);
				z = std::atan2(-f, k);

			}
			else
			{
				y = std::atan2(h, a);
				z = 0;
			}
		}
		else if ("ZYX" == order) {
			y = std::asin(-Clamp(s, -1, 1));

			if (0.999999 > abs(s)) {
				x = std::atan2(n, e);
				z = std::atan2(h, a);
			}
			else
			{
				x = 0;
				z = std::atan2(-f, k);
			}
		}
		else if ("YZX" == order) {
			z = std::asin(Clamp(h, -1, 1));

			if (0.999999 > abs(h)) {
				x = std::atan2(-l, k);
				y = std::atan2(-s, a);

			}
			else
			{
				y = std::atan2(g, e);
				x = 0;
			}
		}
		else if ("XZY" == order) {
			z = std::asin(-Clamp(f, -1, 1));

			if (0.999999 > abs(f)) {
				x = std::atan2(n, k);
				y = std::atan2(g, a);
			}
			else
			{
				x = std::atan2(-l, e);
				y = 0;
			}
		}
		else
		{
			throw std::runtime_error("Bad order!");
		}

		x = to_angle(x);
		y = to_angle(y);
		z = to_angle(z);
		return euler;
	}

}


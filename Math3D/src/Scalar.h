#pragma once
#include <math.h>

namespace m3
{

#ifndef PI
#define PI       3.14159265358979323846
#endif // !PI

	inline float Rad2Deg(float rad) { return rad * 180 / PI; }

	inline float Deg2Rad(float degree) { return degree / 180 * PI; }

	inline float Clamp(float x, float min, float max) { return x > max ? max : x < min ? min : x; }

	inline float Min(float x, float y) { return x < y ? x : y; }

	inline float Max(float x, float y) { return x > y ? x : y; }

	inline float Atan2(float y, float x) { return atan2f(y, x); }
}
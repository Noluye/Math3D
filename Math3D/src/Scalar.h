#pragma once

namespace m3
{

#ifndef PI
#define PI       3.14159265358979323846
#endif // !PI

	inline float to_angle(float x) { return x * 180 / PI; }

	inline float Clamp(float x, float min, float max) { return x > max ? max : x < min ? min : x; }

	inline float Min(float x, float y) { return x < y ? x : y; }

	inline float Max(float x, float y) { return x > y ? x : y; }

}
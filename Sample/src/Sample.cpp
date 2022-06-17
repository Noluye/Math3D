#include <Math3D.h>

using namespace m3;

#define ASSERT(expr, ...) { if(!(expr)) { std::cout << "Assertion Failed: " << __VA_ARGS__ << std::endl;  __debugbreak();} }

void TestOrthoNormalize()
{
	{
		Vec3 v1 = Vec3::Left();
		Vec3 v2 = Vec3::Up();
		OrthoNormalize(v1, v2);
		ASSERT(v2 == Vec3::Up(), "OrthoNormalize faild. [Test 1]");
	}
	
	{
		Vec3 v1 = Vec3::Left();
		Vec3 v2 = {1.0f, 1.0f, 0.0f};
		OrthoNormalize(v1, v2);
		ASSERT(v2 == Vec3::Up(), "OrthoNormalize faild. [Test 2]");
	}

	{
		// TODO: Not figure out now
		Vec3 v1 = { 1.0f, 1.0f, 0.0f };
		Vec3 v2 = { 1.0f, 1.0f, 0.0f };
		OrthoNormalize(v1, v2);
		Vec3 gt = { -0.7071067811865475f, 0.7071067811865475f, 0.0f };
		//ASSERT(v2 == gt, "OrthoNormalize faild. [Test 3]");
		//std::cout << v2 << std::endl;
	}
	{
		// TODO: Not figure out now
		Vec3 v1 = { 1.0f, 2.0f, 3.0f };
		Vec3 v2 = { 4.0f, 5.0f, 6.0f };
		OrthoNormalize(v1, v2);
		Vec3 gt = { -0.7071067811865475f, 0.7071067811865475f, 0.0f };
		//ASSERT(v2 == gt, "OrthoNormalize faild. [Test 3]");
		std::cout << v2 << std::endl;
	}
}

int main()
{
	TestOrthoNormalize();
	
}
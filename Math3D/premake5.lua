project "Math3D"
	kind "StaticLib"
	language "C++"
	cppdialect "C++20"
	staticruntime "On"

	targetdir ("../bin/" .. outputdir .. "/%{prj.name}")
	objdir ("../bin-int/" .. outputdir .. "/%{prj.name}")

	pchheader "m3pch.h"
	pchsource "src/m3pch.cpp"

	files { "src/**.cpp", "src/**.h" }

	includedirs {
		"src"
	}

	filter "system:windows"
		systemversion "latest"
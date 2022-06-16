project "Samples"
	kind "ConsoleApp"
	language "C++"
	cppdialect "C++20"
	staticruntime "On"

	targetdir ("../bin/" .. outputdir .. "/%{prj.name}")
	objdir ("../bin-int/" .. outputdir .. "/%{prj.name}")

	files { "src/**.cpp", "src/**.h", "premake5.lua", "../premake5.lua", "../README.md" }

	includedirs {
		"src",
		"../Math3D/src"
	}

	links { "Math3D" }

	filter "system:windows"
		systemversion "latest"
		libdirs {"../bin/" .. outputdir .. "/Math3D"}
		dependson { "Math3D" }
		links { "Math3D.lib" }

	filter{}
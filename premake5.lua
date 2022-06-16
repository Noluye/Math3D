workspace "Math3D"
	configurations { "Debug", "Release", "Dist" }
	platforms "x64"
	architecture "x86_64"
	startproject "Sample"

	filter "configurations:Debug"
		defines { "DEBUG" }
		symbols "On"

	filter "configurations:Release"
		defines { "NDEBUG" }
		optimize "On"
		symbols "Off"

	filter "configurations:Dist"
		defines { "NDEBUG" }
		optimize "On"
		symbols "Off"

	outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

	include "Math3D"
	include "Sample"

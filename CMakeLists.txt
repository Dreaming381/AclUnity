cmake_minimum_required(VERSION 3.12)

project(AclUnity)

set(ACLUNITY_USE_ACL_SUBDIR false CACHE BOOL "Add ACL via add_subdirectory which allows code parsing of tools")
set(ACLUNITY_IS_AVX false CACHE BOOL "Use AVX, AVX2, BMI1, and BMI2 instructions plus SSE4 tier")
set(ACLUNITY_IS_SSE4 false CACHE BOOL "Use SSE4.2 plus popcnt")

file(GLOB_RECURSE src CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cpp")
#The following helps IDEs show these files in the project.
file(GLOB_RECURSE apiHeaders "${PROJECT_SOURCE_DIR}/include*.h")

if (ACLUNITY_USE_ACL_SUBDIR)
	add_subdirectory(acl)
	set(aclHeaders "")
else()
	file(GLOB_RECURSE aclHeaders "${PROJECT_SOURCE_DIR}/acl/includes**.h")
endif()

if (MSVC)
	if (ACLUNITY_IS_AVX)
		add_library(AclUnity_AVX SHARED ${src} ${apiHeaders} ${aclHeaders})
		target_compile_options(AclUnity_AVX PUBLIC "/arch:AVX2" "/FA")
		target_compile_definitions(AclUnity_AVX PUBLIC ACL_USE_POPCOUNT)
		target_compile_features(AclUnity_AVX PUBLIC cxx_std_11)
		target_include_directories(AclUnity_AVX PUBLIC "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/acl/includes" "${PROJECT_SOURCE_DIR}/acl/external/rtm/includes")
	elseif (ACLUNITY_IS_SSE4)
		add_library(AclUnity_SSE4 SHARED ${src} ${apiHeaders} ${aclHeaders})
		target_compile_options(AclUnity_SSE4 PUBLIC "/d2archSSE42" "/FA")
		target_compile_definitions(AclUnity_SSE4 PUBLIC DACL_USE_POPCOUNT)
		target_compile_features(AclUnity_SSE4 PUBLIC cxx_std_11)
		target_include_directories(AclUnity_SSE4 PUBLIC "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/acl/includes" "${PROJECT_SOURCE_DIR}/acl/external/rtm/includes")
	else()
		add_library(AclUnity SHARED ${src} ${apiHeaders} ${aclHeaders})
		# SSE2 is assumed for MSVC x64
		target_compile_options(AclUnity PUBLIC "/FA")
		target_compile_features(AclUnity PUBLIC cxx_std_11)
		target_include_directories(AclUnity PUBLIC "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/acl/includes" "${PROJECT_SOURCE_DIR}/acl/external/rtm/includes")
	endif()
endif()




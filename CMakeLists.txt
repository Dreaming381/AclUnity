cmake_minimum_required(VERSION 3.14)

project(AclUnity)

include(CheckCXXCompilerFlag)

set(ACLUNITY_USE_ACL_SUBDIR false CACHE BOOL "Add ACL via add_subdirectory which allows code parsing of tools")
set(ACLUNITY_IS_AVX false CACHE BOOL "Use AVX, AVX2, BMI1, and BMI2 instructions plus SSE4 tier")
set(ACLUNITY_IS_SSE4 false CACHE BOOL "Use SSE4.2 plus popcnt")

set(ACLUNITY_KEEP_FULL_SYMBOLS false CACHE BOOL "Disables splitting debug symbols into a separate file on some platforms (Linux and Android)")
set(ACLUNITY_MAKE_DEBUG false CACHE BOOL "Make a debug executable")
set(ACLUNITY_MAKE_TESTS false CACHE BOOL "Make the determinism test executable")

file(GLOB_RECURSE src CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cpp")
#The following helps IDEs show these files in the project.
file(GLOB_RECURSE apiHeaders "${PROJECT_SOURCE_DIR}/include*.h")
file(GLOB_RECURSE srcHeaders "${PROJECT_SOURCE_DIR}/src/*.h")

if (ACLUNITY_USE_ACL_SUBDIR)
	add_subdirectory(acl)
	set(aclHeaders "")
else()
	file(GLOB_RECURSE aclHeaders "${PROJECT_SOURCE_DIR}/acl/includes**.h")
endif()

if (APPLE AND NOT CMAKE_SYSTEM_NAME STREQUAL "iOS" AND (CMAKE_OSX_ARCHITECTURES STREQUAL "arm64"))
	set(CMAKE_OSX_DEPLOYMENT_TARGET 11.0)
endif()

if (MSVC)
	# CMake's default for RelWithDebInfo links incrementally, which disables /OPT:REF and /OPT:ICF and pads every function.
	set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "/debug /INCREMENTAL:NO /OPT:REF /OPT:ICF")
	set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "/debug /INCREMENTAL:NO /OPT:REF /OPT:ICF")
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "iOS")
	add_library(AclUnity STATIC ${src} ${apiHeaders} ${srcHeaders} ${aclHeaders})
else()
	add_library(AclUnity SHARED ${src} ${apiHeaders} ${srcHeaders} ${aclHeaders})
endif()
target_include_directories(AclUnity PUBLIC "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/acl/includes" "${PROJECT_SOURCE_DIR}/acl/external/rtm/includes")
target_compile_features(AclUnity PUBLIC cxx_std_11)
target_compile_definitions(AclUnity PRIVATE ACL_UNITY_BUILD)
set_target_properties(AclUnity PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)

# The deterministic decompression functions require that the compiler never fuses a multiply and an add.
if (MSVC)
	set_source_files_properties("${PROJECT_SOURCE_DIR}/src/decompression_deterministic.cpp" PROPERTIES COMPILE_OPTIONS "/fp:precise")
else()
	set_source_files_properties("${PROJECT_SOURCE_DIR}/src/decompression_deterministic.cpp" PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-fno-fast-math")
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	# Unity does not guarantee libstdc++ is loaded or recent enough, so link it statically and keep its symbols private.
	# --no-undefined turns any unresolved symbol into a link error instead of a load failure on the user's machine.
	target_link_options(AclUnity PRIVATE "-static-libstdc++" "-Wl,--exclude-libs,ALL" "-Wl,--no-undefined")
endif()

if (UNIX AND NOT APPLE AND NOT ACLUNITY_KEEP_FULL_SYMBOLS)
	add_custom_command(TARGET AclUnity POST_BUILD
					COMMAND ${CMAKE_OBJCOPY} "--only-keep-debug" "$<TARGET_FILE:AclUnity>" "$<TARGET_FILE:AclUnity>.debug"
					COMMAND ${CMAKE_STRIP} "--strip-debug" "$<TARGET_FILE:AclUnity>"
					COMMAND ${CMAKE_OBJCOPY} "--add-gnu-debuglink=$<TARGET_FILE:AclUnity>.debug" "$<TARGET_FILE:AclUnity>")
endif()

if (MSVC)
	set(CMAKE_SHARED_LIBRARY_PREFIX "")
	if (ACLUNITY_IS_AVX)
		set_target_properties(AclUnity PROPERTIES OUTPUT_NAME "AclUnity_AVX")
		target_compile_options(AclUnity PUBLIC "/arch:AVX2" "/FA")
		target_compile_definitions(AclUnity PUBLIC ACL_USE_POPCOUNT)
	elseif (ACLUNITY_IS_SSE4)
		set_target_properties(AclUnity PROPERTIES OUTPUT_NAME "AclUnity_SSE4")
		target_compile_options(AclUnity PUBLIC "/d2archSSE42" "/FA")
		target_compile_definitions(AclUnity PUBLIC ACL_USE_POPCOUNT)
	else()
		# SSE2 is assumed for MSVC x64
		target_compile_options(AclUnity PUBLIC "/FA")
	endif()
elseif (APPLE AND (CMAKE_OSX_ARCHITECTURES STREQUAL "arm64"))
		set(CMAKE_SHARED_LIBRARY_PREFIX "")
		set(CMAKE_STATIC_LIBRARY_PREFIX "")
		target_compile_options(AclUnity PUBLIC "--save-temps=obj")
elseif (ANDROID)
		set_target_properties( AclUnity PROPERTIES OUTPUT_NAME "AclUnity")
		target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-Wl,-z,max-page-size=16384")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
		target_compile_options(AclUnity PUBLIC "--save-temps=obj")
elseif (UNIX OR APPLE)
	if (ACLUNITY_IS_AVX)
		set_target_properties(AclUnity PROPERTIES OUTPUT_NAME "AclUnity_AVX")
		target_compile_options(AclUnity PUBLIC "-march=x86-64-v3" "--save-temps=obj" "-masm=intel")
		target_compile_definitions(AclUnity PUBLIC ACL_USE_POPCOUNT)
	elseif (ACLUNITY_IS_SSE4)
		set_target_properties(AclUnity PROPERTIES OUTPUT_NAME "AclUnity_SSE4")
		target_compile_options(AclUnity PUBLIC "-march=x86-64-v2" "--save-temps=obj" "-masm=intel")
		target_compile_definitions(AclUnity PUBLIC ACL_USE_POPCOUNT)
	else()
		target_compile_options(AclUnity PUBLIC "-march=x86-64" "--save-temps=obj" "-masm=intel")
	endif()
endif()

# Workaround for LLVM 19+ until ACL is updated accordingly
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	check_cxx_compiler_flag("-Wno-missing-template-arg-list-after-template-kw" ACLUNITY_HAS_TEMPLATE_KW_WARNING)
	if (ACLUNITY_HAS_TEMPLATE_KW_WARNING)
		target_compile_options(AclUnity PUBLIC "-Wno-missing-template-arg-list-after-template-kw")
	endif()
endif()

if (ACLUNITY_MAKE_DEBUG)
	add_executable(debug_main "tests/debug_main.cpp")
	target_link_libraries(debug_main AclUnity)
	target_compile_definitions(AclUnity PUBLIC ACL_ON_ASSERT_THROW)
endif()

if (ACLUNITY_MAKE_TESTS)
	add_executable(determinism_test "tests/determinism_test.cpp")
	target_link_libraries(determinism_test AclUnity)
	if (MSVC)
		target_compile_definitions(determinism_test PRIVATE _CRT_SECURE_NO_WARNINGS)
	endif()
endif()

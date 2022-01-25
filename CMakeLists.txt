cmake_minimum_required(VERSION 3.12)

project(AclUnity)

add_subdirectory(acl)

file(GLOB_RECURSE src CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cpp")
#The following helps IDEs show these files in the project.
file(GLOB_RECURSE apiHeaders "${PROJECT_SOURCE_DIR}/include*.h")
add_library(AclUnity SHARED ${src} ${apiHeaders})

target_compile_features(AclUnity PUBLIC cxx_std_11)

target_include_directories(AclUnity PUBLIC "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/acl/includes" "${PROJECT_SOURCE_DIR}/acl/external/rtm/includes")



cmake_minimum_required(VERSION 3.1)
project(sbodies)

message( "Using ${CMAKE_CXX_COMPILER_ID} c++ compiler" )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")

file(GLOB_RECURSE sources src/*.cpp)
file(GLOB_RECURSE ext ext/*.cpp)

include_directories(src)
include_directories(ext/imgui)
link_directories(libs)

add_executable(sbodies ${sources} ${ext})

target_link_libraries(sbodies GL)
target_link_libraries(sbodies sfml-graphics)
target_link_libraries(sbodies sfml-window)
target_link_libraries(sbodies sfml-system)

set(CMAKE_BUILD_TYPE Debug)
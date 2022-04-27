cmake_minimum_required(VERSION 3.6)

project(railbot)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Bin)
set (CMAKE_CXX_FLAGS  "-std=c++17 -w -march=silvermont -mtune=silvermont -Og")
set(CMAKE_EXE_LINKER_FLAGS " -no-pie")

link_directories(
    Lib/
)

add_executable(test Src/test.cpp)
target_link_libraries(test -leigen-quadprog)
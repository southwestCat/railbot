cmake_minimum_required(VERSION 3.6)

project(railbot)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Bin)
set (CMAKE_CXX_FLAGS  "-std=c++17 -Wall -Wno-cast-function-type -Wno-unused-variable -march=silvermont -mtune=silvermont -Og")
# set (CMAKE_CXX_FLAGS  "-std=c++17 -w -march=silvermont -mtune=silvermont -O3")
set(CMAKE_EXE_LINKER_FLAGS " -no-pie")

include_directories(
    Src/
    Src/CPPController
    Src/LolaConnector/msgpack/include
)

add_executable(lola_conn 
    Src/LolaConnector/lola_conn.cpp
    Src/LolaConnector/leds.cpp
    Src/LolaConnector/lola_frame.cpp
)
target_link_libraries(lola_conn -lboost_system -lpthread -lrt)

file(GLOB_RECURSE CONTROLLER_SRC Src/CPPController/[^.]*.cpp)
add_executable(controller ${CONTROLLER_SRC})
target_link_libraries(controller -pthread -leigen-quadprog -lrt)

add_executable(test Src/test.cpp)
target_link_libraries(test -leigen-quadprog)
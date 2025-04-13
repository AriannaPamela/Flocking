cmake_minimum_required(VERSION 3.28)

project(APF_flocking)

set(CMAKE_CXX_STANDARD 20)

add_executable(APF_flocking main.cpp)

find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)

add_executable(FlockingProject  main.cpp)

target_link_libraries(APF_flocking sfml-system sfml-window sfml-graphics)
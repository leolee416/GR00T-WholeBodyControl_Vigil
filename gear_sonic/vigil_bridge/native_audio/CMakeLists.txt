cmake_minimum_required(VERSION 3.16)
project(g1_vigil_led_speaker_runner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(unitree_sdk2 REQUIRED)

add_executable(
  g1_vigil_led_speaker_runner
  g1_vigil_led_speaker_runner.cpp
)
target_link_libraries(g1_vigil_led_speaker_runner PRIVATE unitree_sdk2)

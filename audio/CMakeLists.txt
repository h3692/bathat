cmake_minimum_required(VERSION 3.16)

project(AudioRadar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    audio_radar_test
    src/main.cpp
    src/AudioManager.cpp
    src/WavAudioOutput.cpp
)

target_include_directories(
    audio_radar_test
    PRIVATE
    include
)

target_compile_options(
    audio_radar_test
    PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
cmake_minimum_required(VERSION 3.22)
project(H7_BSP_WS2812_Tests LANGUAGES CXX)
get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_executable(ws2812_tests
    ws2812_tests.cpp
    ${BSP_ROOT}/User_File/Device/Onboard/WS2812/bsp_ws2812.cpp
)
target_compile_features(ws2812_tests PRIVATE cxx_std_17)
target_include_directories(ws2812_tests PRIVATE
    stubs
    ${BSP_ROOT}/User_File/Device/Onboard/WS2812
)
enable_testing()
add_test(NAME ws2812_changed_color COMMAND ws2812_tests)

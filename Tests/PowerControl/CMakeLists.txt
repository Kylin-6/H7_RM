cmake_minimum_required(VERSION 3.22)
project(H7_BSP_PowerControl_Tests LANGUAGES CXX)

get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")
set(APPLICATION "${BSP_ROOT}/User_File/Application/PowerControl")

add_executable(power_control_tests
    power_control_tests.cpp
    ${APPLICATION}/power_control.cpp
    ${ALGORITHM}/RLS/alg_rls.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
)
target_compile_features(power_control_tests PRIVATE cxx_std_17)
target_include_directories(power_control_tests PRIVATE
    ${BSP_ROOT}/Tests/Boundary/stubs
    ${APPLICATION}
    ${ALGORITHM}/RLS
    ${ALGORITHM}/Basic
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(power_control_tests PRIVATE -Wall -Wextra -Werror=return-type)
elseif(MSVC)
    target_compile_options(power_control_tests PRIVATE /utf-8)
endif()

enable_testing()
foreach(group limit identification invalid)
    add_test(NAME power_control_${group} COMMAND power_control_tests ${group})
endforeach()

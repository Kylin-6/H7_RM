cmake_minimum_required(VERSION 3.22)
project(H7_BSP_Health_Tests LANGUAGES CXX)
get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")
set(DJI "${BSP_ROOT}/User_File/Device/Peripheral/Motor/DJImotor")
add_executable(health_tests
    health_tests.cpp
    ${BSP_ROOT}/User_File/System/Health/sys_health.cpp
    ${BSP_ROOT}/User_File/Task/StatusTask.cpp
    ${DJI}/dji_motor.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
    ${ALGORITHM}/PID/alg_pid.cpp
    ${ALGORITHM}/Filter/IIR/alg_filter_iir.cpp
)
target_compile_features(health_tests PRIVATE cxx_std_17)
target_include_directories(health_tests PRIVATE
    stubs ../Boundary/stubs
    ${BSP_ROOT}/User_File/System/Health
    ${BSP_ROOT}/User_File/System/debug
    ${DJI} ${ALGORITHM}/Basic ${ALGORITHM}/PID
    ${ALGORITHM}/Filter/IIR
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(health_tests PRIVATE -Wall -Wextra -Werror=return-type)
endif()
enable_testing()
foreach(group contract imu motor capacity task)
    add_test(NAME health_${group} COMMAND health_tests ${group})
endforeach()

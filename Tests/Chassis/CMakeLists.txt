cmake_minimum_required(VERSION 3.22)
project(H7_Chassis_Safety_Tests LANGUAGES CXX)
get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_executable(chassis_safety_tests chassis_safety_tests.cpp
    ${ROOT}/User_File/Application/Chassis/Chassis.cpp
    ${ROOT}/User_File/System/MessageCenter/message_center.cpp)
target_compile_definitions(chassis_safety_tests PRIVATE CHASSIS=1)
target_include_directories(chassis_safety_tests PRIVATE stubs
    ${ROOT}/Tests/Gimbal/stubs ${ROOT}/Tests/Boundary/stubs
    ${ROOT}/User_File/Application/Chassis
    ${ROOT}/User_File/System/MessageCenter)
enable_testing()
add_test(NAME chassis_safety COMMAND chassis_safety_tests)

cmake_minimum_required(VERSION 3.22)
project(H7_Transport_Tests LANGUAGES CXX)
get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_executable(transport_tests transport_tests.cpp
    ${ROOT}/User_File/System/Transport/transport_protocol.cpp
    ${ROOT}/User_File/System/Transport/transport_chassis.cpp)
target_include_directories(transport_tests PRIVATE stubs
    ${ROOT}/User_File/System/Transport
    ${ROOT}/User_File/System/MessageCenter)
enable_testing()
add_test(NAME transport COMMAND transport_tests)
add_executable(gimbal_feedback_tests gimbal_feedback_tests.cpp
    ${ROOT}/User_File/System/Transport/transport_protocol.cpp
    ${ROOT}/User_File/System/Transport/transport_gimbal.cpp)
target_include_directories(gimbal_feedback_tests PRIVATE stubs
    ${ROOT}/User_File/System/Transport
    ${ROOT}/User_File/System/MessageCenter)
add_test(NAME gimbal_feedback COMMAND gimbal_feedback_tests)

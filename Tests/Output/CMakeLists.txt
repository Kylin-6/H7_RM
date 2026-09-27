cmake_minimum_required(VERSION 3.22)
project(H7_Output_Tests LANGUAGES CXX)
get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_executable(output_tests output_tests.cpp
    ${ROOT}/User_File/Application/RobotCmd/RobotCmd.cpp
    ${ROOT}/User_File/System/MessageCenter/message_center.cpp)
target_include_directories(output_tests PRIVATE
    ${ROOT}/User_File/Application/RobotCmd
    ${ROOT}/User_File/System/MessageCenter
    ${ROOT}/Tests/Gimbal/stubs
    ${ROOT}/Tests/Boundary/stubs)
enable_testing()
add_test(NAME output COMMAND output_tests)

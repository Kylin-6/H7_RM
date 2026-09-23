cmake_minimum_required(VERSION 3.22)
project(H7_BSP_SpinMode_Tests LANGUAGES CXX)

if(NOT BSP_ROOT)
    get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
endif()
if(NOT SPIN_MODE_SOURCE_DIR)
    set(SPIN_MODE_SOURCE_DIR "${BSP_ROOT}/User_File/Application/SpinMode")
endif()
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")

add_executable(spin_mode_tests
    spin_mode_tests.cpp
    ${SPIN_MODE_SOURCE_DIR}/spin_mode.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
    ${ALGORITHM}/Matrix/alg_matrix.cpp
)
target_compile_features(spin_mode_tests PRIVATE cxx_std_17)
target_include_directories(spin_mode_tests PRIVATE
    ${BSP_ROOT}/Tests/Boundary/stubs
    ${SPIN_MODE_SOURCE_DIR}
    ${ALGORITHM}/Basic
    ${ALGORITHM}/Matrix
    ${ALGORITHM}/FSM
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(spin_mode_tests PRIVATE -Wall -Wextra -Werror=return-type)
endif()

enable_testing()
foreach(group transform angular modes lifecycle)
    add_test(NAME spin_mode_${group} COMMAND spin_mode_tests ${group})
endforeach()

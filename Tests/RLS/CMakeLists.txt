cmake_minimum_required(VERSION 3.22)
project(H7_BSP_RLS_Tests LANGUAGES CXX)

get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")

add_executable(rls_tests
    rls_tests.cpp
    ${ALGORITHM}/RLS/alg_rls.cpp
)
target_compile_features(rls_tests PRIVATE cxx_std_11)
target_include_directories(rls_tests PRIVATE ${ALGORITHM}/RLS)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(rls_tests PRIVATE -Wall -Wextra -Werror)
elseif(MSVC)
    target_compile_options(rls_tests PRIVATE /utf-8)
endif()

enable_testing()
add_test(NAME rls_contract COMMAND rls_tests contract)
add_test(NAME rls_analytic COMMAND rls_tests analytic)
add_test(NAME rls_convergence COMMAND rls_tests convergence)
add_test(NAME rls_forgetting COMMAND rls_tests forgetting)
add_test(NAME rls_numerical_failure COMMAND rls_tests numerical_failure)
add_test(NAME rls_positive_definite COMMAND rls_tests positive_definite)

cmake_minimum_required(VERSION 3.22)
project(H7_Daemon_Tests LANGUAGES C CXX)
get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(USER "${ROOT}/User_File")
set(ALG "${USER}/Middleware/Algorithm")
set(REF "${USER}/Device/Peripheral/Referee")
add_executable(daemon_tests daemon_tests.cpp
    ${USER}/System/Daemon/daemon.cpp
    ${USER}/System/IMU/sys_imu.cpp
    ${USER}/System/MessageCenter/message_center.cpp
    ${USER}/Device/Peripheral/Motor/DJImotor/dji_motor.cpp
    ${USER}/Device/Peripheral/Remote/sbus.cpp
    ${USER}/Application/Input/input_state.cpp
    ${USER}/Application/Input/remote_input.cpp
    ${USER}/Application/Input/source_arbitration.cpp
    ${REF}/referee_26.c ${REF}/vtm_26.c ${REF}/crc_ref.c ${REF}/referee_daemon.cpp
    ${ALG}/Basic/alg_basic.cpp ${ALG}/Matrix/alg_matrix.cpp
    ${ALG}/PID/alg_pid.cpp ${ALG}/Filter/IIR/alg_filter_iir.cpp)
target_compile_features(daemon_tests PRIVATE cxx_std_17)
target_include_directories(daemon_tests PRIVATE ${USER}/System/Daemon stubs
    ../Gimbal/stubs ../Boundary/stubs
    ${USER}/System/Daemon ${USER}/System/IMU ${USER}/System/MessageCenter
    ${USER}/Device/Peripheral/Motor/DJImotor ${USER}/Device/Peripheral/Remote
    ${USER}/Application/Input ${REF}
    ${ALG}/Basic ${ALG}/Matrix ${ALG}/PID ${ALG}/Filter/IIR)
target_compile_options(daemon_tests PRIVATE -Wall -Wextra -Werror=return-type)
enable_testing()
foreach(group transitions capacity dji dji_registration sbus_input ins referee vtm registration)
    add_test(NAME daemon_${group} COMMAND daemon_tests ${group})
    set_tests_properties(daemon_${group} PROPERTIES TIMEOUT 10)
endforeach()

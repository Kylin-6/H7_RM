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
    ${USER}/Device/Peripheral/Motor/DMmotor/dmmotor.cpp
    ${USER}/Device/Peripheral/IMU/DM_IMU/dvc_dm_imu.cpp
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
    ${USER}/Device/Peripheral/Motor/DJImotor ${USER}/Device/Peripheral/Motor/DMmotor
    ${USER}/Device/Peripheral/IMU/DM_IMU ${USER}/Device/Peripheral/Remote
    ${USER}/Application/Input ${REF}
    ${ALG}/Basic ${ALG}/Matrix ${ALG}/PID ${ALG}/Filter/IIR)
target_compile_options(daemon_tests PRIVATE -Wall -Wextra -Werror=return-type)
enable_testing()
foreach(group transitions capacity dji dji_registration dm dm_imu sbus_input ins referee vtm registration)
    add_test(NAME daemon_${group} COMMAND daemon_tests ${group})
    set_tests_properties(daemon_${group} PROPERTIES TIMEOUT 10)
endforeach()

# 老步兵云台板（LEGACY_INFANTRY_GIMBAL）路径：0x065 链路、DM-IMU 桥与诊断映射。
add_executable(legacy_input_tests legacy_input_tests.cpp
    ${USER}/System/Daemon/daemon.cpp
    ${USER}/System/MessageCenter/message_center.cpp
    ${USER}/Device/Peripheral/ChassisBoard/chassis_board.cpp
    ${USER}/Device/Peripheral/IMU/DM_IMU/dvc_dm_imu.cpp
    ${USER}/Device/Peripheral/IMU/DM_IMU/dm_imu_ins.cpp
    ${USER}/Application/Diagnostics/Diagnostics.cpp
    ${USER}/Application/Input/input_state.cpp
    ${USER}/Application/Input/remote_input.cpp
    ${ALG}/Basic/alg_basic.cpp ${ALG}/Filter/IIR/alg_filter_iir.cpp)
target_compile_features(legacy_input_tests PRIVATE cxx_std_17)
target_compile_definitions(legacy_input_tests PRIVATE LEGACY_INFANTRY_GIMBAL=1)
target_include_directories(legacy_input_tests PRIVATE
    ${USER}/System/Daemon legacy_stubs ../Boundary/stubs
    ${USER}/System/MessageCenter
    ${USER}/Device/Peripheral/ChassisBoard ${USER}/Device/Peripheral/IMU/DM_IMU
    ${USER}/Application/Diagnostics ${USER}/Application/Input
    ${USER}/Application/Gimbal ${ROOT}/User_Config/Board
    ${ALG}/Basic ${ALG}/Filter/IIR)
target_compile_options(legacy_input_tests PRIVATE -Wall -Wextra -Werror=return-type)
foreach(group chassis_link remote_link bridge_freshness diagnostics)
    add_test(NAME legacy_${group} COMMAND legacy_input_tests ${group})
    set_tests_properties(legacy_${group} PROPERTIES TIMEOUT 10)
endforeach()

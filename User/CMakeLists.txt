target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Hardware/Display/src/oled.c
    Hardware/Imu/src/hwt101.c
    Hardware/Imu/src/jy901s.c
    Hardware/Imu/src/imu_device.c
    Hardware/Indicator/src/board_led.c
    Hardware/Input/src/switch_input.c
    Hardware/Motor/src/M2006.c
    Hardware/Serial/src/uart_port.c
    Hardware/Servo/src/servo_pwm.c
    Runtime/Diagnostics/src/imu_diagnostics_adapter.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/Display/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/Imu/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/Indicator/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/Input/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/Motor/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/Serial/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/Servo/inc
)

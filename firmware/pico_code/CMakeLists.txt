cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)


# --------------------------------------------------------------------------------------------------
# --- NEW TARGET: SLAMBOT_HARDWARE_DRIVER  -------
# --------------------------------------------------------------------------------------------------

add_executable(slambot_hardware_driver
    slambot_hardware_driver.c
    pico_uart_transport.c
    bno055.c
    imu_bno055_driver.c
)
target_link_libraries(slambot_hardware_driver
    pico_stdlib
    microros
    hardware_pwm
    hardware_i2c 
    hardware_gpio
    hardware_irq
    pico_time
)
target_include_directories(slambot_hardware_driver PUBLIC
    libmicroros/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_pwm/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Configure Pico stdio for waveshare target and generate UF2
pico_enable_stdio_usb(slambot_hardware_driver 1)
pico_enable_stdio_uart(slambot_hardware_driver 0)
pico_add_extra_outputs(slambot_hardware_driver)



# ----------------------------------------------------------------------
# --- ORIGINAL TARGET: PICO_MICRO_ROS_EXAMPLE (Retained) -----------------
# ----------------------------------------------------------------------


# add_executable(pico_micro_ros_example
#     pico_micro_ros_example.c
#     pico_uart_transport.c
# )
# target_link_libraries(pico_micro_ros_example
#     pico_stdlib
#     microros
# )

# target_include_directories(pico_micro_ros_example PUBLIC
#     libmicroros/include
# )

# # Configure Pico and Generate UF2 for the original example
# pico_enable_stdio_usb(pico_micro_ros_example 1)
# pico_enable_stdio_uart(pico_micro_ros_example 0)
# pico_add_extra_outputs(pico_micro_ros_example)



# ----------------------------------------------------------------------
# --- COMMON COMPILE FLAGS (At the end, affecting all targets) -----------
# ----------------------------------------------------------------------

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
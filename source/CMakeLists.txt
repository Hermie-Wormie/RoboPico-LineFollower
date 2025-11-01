# ==============================================================
# CMake for source modules (included by root CMakeLists)
# ==============================================================

# Collect all source files for the robot system
target_sources(${PROJECT_NAME} PRIVATE
    io/blink.c
    io/io_handler.c
    io/line_following.c
    io/barcodes.c
    io/station1.c
    networking/wifi.c
    sensors/interrupt.c
    sensors/encoder.c
    sensors/ultrasonic.c
    sensors/infrared.c
    hmc5883l/hmc5883l.c
    motor/motor.c
)

# (Optional) If your code uses extra include directories not in root
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/io
    ${CMAKE_CURRENT_LIST_DIR}/motor
    ${CMAKE_CURRENT_LIST_DIR}/hmc5883l
    ${CMAKE_CURRENT_LIST_DIR}/networking
    ${CMAKE_CURRENT_LIST_DIR}/sensors
)

# ==============================================================
# Top-level CMake for Pico + FreeRTOS Project (Pico W)
# ==============================================================

cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Board setup ---
set(PICO_BOARD pico_w CACHE STRING "Board type")

# --- Path to FreeRTOS Kernel ---
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)

# --- Import Pico SDK ---
include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(FreeRTOS_Project C CXX ASM)

# --- Initialize SDK ---
pico_sdk_init()

# --- Source files ---
add_executable(${PROJECT_NAME}
    main.c
    # motor_straight_test.c
)

# --- Add source directory (your modules) ---
add_subdirectory(source)

# --- Include directories ---
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}                            # Root (for FreeRTOSConfig.h)
    ${CMAKE_CURRENT_LIST_DIR}/source
    ${CMAKE_CURRENT_LIST_DIR}/source/io
    ${CMAKE_CURRENT_LIST_DIR}/source/motor
    ${CMAKE_CURRENT_LIST_DIR}/source/networking
    ${CMAKE_CURRENT_LIST_DIR}/source/sensors
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
)

# --- Link libraries ---
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    FreeRTOS-Kernel-Heap4
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_pwm
    hardware_adc
)

# --- Enable USB output, disable UART ---
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# --- Build output ---
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

pico_add_extra_outputs(${PROJECT_NAME})

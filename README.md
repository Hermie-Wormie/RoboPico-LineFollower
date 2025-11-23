# 🤖 RoboPico-LineFollower

**Autonomous Line-Following Robot Powered by Raspberry Pi Pico**

###
* **Real-Time Control:** Powered by **FreeRTOS**, tasks for sensor reading, motor control, and control loop execution are managed efficiently.
* **Multi-Sensor Fusion:** Utilizes **Infrared (IR)** sensors for line tracking, **Ultrasonic** sensors for obstacle detection, and the **IMU Magnetometer** for accurate heading and **Barcode Navigation**.
* **Motor Feedback:** Includes **Encoder** support (`encoder.c`) for precise velocity control and distance measurement.
* **Networking & Telemetry:** Integrated **Wi-Fi** capabilities (`wifi.c`, `telemetry.h`) for remote monitoring, status reporting, or configuration adjustments.

### Prerequisites
1.  **Pico SDK:** Ensure the Raspberry Pi Pico SDK is installed and configured on your machine.
2.  **GCC Compiler:** Requires the appropriate ARM cross-compiler for the Pico (e.g., `arm-none-eabi-gcc`).
3.  **CMake:** Necessary for building the project and managing dependencies.
4.  **FreeRTOS:** For task scheduling.

   RoboPico-LineFollower/
├── source/
│   ├── hmc5883l/
│   │   ├── hmc5883l.c    # Driver for the HMC5883L Magnetometer (Compass).
│   │   └── hmc5883l.h
│   ├── io/
│   │   ├── barcodes.c    # Logic for reading or processing track-side barcodes/markers for advanced navigation.
│   │   ├── blink.c       # Simple LED blinking utility for debugging and status signaling.
│   │   ├── io_handler.c  # General Input/Output initialization and management routines.
│   │   └── line_follow.c # The core **Line-Following Control Algorithm** (e.g., PID loop, state machine).
│   ├── motor/
│   │   ├── motor.c       # Functions for motor speed control (PWM generation) and direction control.
│   │   └── motor.h
│   ├── networking/
│   │   ├── config.h      # Header defining static Wi-Fi credentials and network configuration settings.
│   │   └── wifi.c        # Implementation of the Wi-Fi stack for communication.
│   ├── sensors/
│   │   ├── encoder.c     # Motor encoder reading, velocity calculation, and distance processing.
│   │   ├── infrared.c    # IR sensor data acquisition, normalization, and filtering.
│   │   ├── ultrasonic.c  # Driver for Ultrasonic distance measurement (e.g., obstacle detection).
│   │   └── sensors.h     # Unified header file for accessing various sensor data.
│   ├── station1.c        # Application-specific logic for navigating to and completing a specific goal or 'station'.
│   └── telemetry.h       # Definition of data structures and packets for Wi-Fi-based data transmission/telemetry.
├── FreeRTOSConfig.h      # Configuration file for the **FreeRTOS** real-time operating system scheduler.
├── CMakeLists.txt        # Top-level build file used by CMake to compile the project.
└── main.c                # Program entry point, where FreeRTOS tasks are created and the scheduler is started.

### How to Build
1. Run CMAKE to generate build files either by '''bash''' or Visual Studio Code Extension
2. Once '.uf2' file is generated, enable BOOTSEL and drag and drop into the RPI-RP2 Drive
3. The robot will run the Line Following Tasks
4. To enable telemetry, run /tele/gateway.py in a console. Make sure that Hotspot IP address aligns with your own
5. Connect your PC and the Pico to Wi-Fi
6. View the telemetry via assigned IP address and port

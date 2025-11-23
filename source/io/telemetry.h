#include <stdbool.h>

typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOWING,
    STATE_SCANNING_BARCODE,
    STATE_MANEUVERING,
    STATE_OBSTACLE_AVOIDING,
    STATE_RECOVERY // The 'Recovery Status' flag
} robot_state_t;

// Structure to hold all telemetry data
typedef struct {
    // 1. Distance travelled (encoder)
    float total_distance_cm;

    // Live Speed (Requested: Live speed data)
    float live_speed_cmps;
    
    // Current State (Requested: Current state)
    robot_state_t current_state; 

    // Line Position (Requested: Real-time line position)
    int16_t line_position_error; 
    
    // Chosen Path (Requested: Clearance path/Chosen path)
    int8_t chosen_path; // -1=Right, 0=Blocked, 1=Left

    // 2. Object distance (ultrasonic)
    float object_distance_cm;

    // 3. Whether object is detected (ultrasonic)
    bool object_detected; // true/false

    // 4. Approximate width of object (ultrasonic)
    float object_width_cm;

    // 5. Barcode command
    char last_barcode_command; // e.g., 'L', 'R', 'S', '\0'

    // 6. IMU filtered data (hmc5883l)
    float filtered_heading_deg; // Final, filtered heading
    float mag_x_filt;           // Filtered magnetic X (optional for raw data)
    float mag_y_filt;           // Filtered magnetic Y
    float mag_z_filt;           // Filtered magnetic Z

} telemetry_data_t;

// Global instance of the telemetry data structure
extern telemetry_data_t g_telemetry_data;
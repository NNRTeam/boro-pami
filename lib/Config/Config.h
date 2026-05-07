#ifndef CONFIG_H
#define CONFIG_H

namespace config {

    bool constexpr TEST = false; // Enable or disable test mode
    bool constexpr MOTOR_ODOM_ONLY = true; // Enable or disable motor odometry only mode

    // HARDWARE CONFIGURATION

    int constexpr M_EN_PIN = 2;

    int constexpr M2_DIR_PIN = 12;
    int constexpr M2_STEP_PIN = 8;

    int constexpr M1_DIR_PIN = 7;
    int constexpr M1_STEP_PIN = 4;

    int constexpr OD1_CS_PIN = 10;
    int constexpr OD2_CS_PIN = 9;

    int constexpr MICROSTEPS = 16; // Microsteps for motor driver (1, 2, 4, 8, 16)
    int constexpr STEP_PER_REVOLUTION = 200; // Steps per revolution for the motor

    // MECHANICAL CONFIGURATION

    float constexpr M1_WHEEL_DIAMETER_MM = 57.8f; // Motor Wheel diameter in millimeters
    float constexpr M2_WHEEL_DIAMETER_MM = 57.8f; // Motor Wheel diameter in millimeters

    bool constexpr M1_INVERT_DIR = false; // Invert motor 1 direction (set true if wheel goes backward)
    bool constexpr M2_INVERT_DIR = true;  // Invert motor 2 direction (set true if wheel goes backward)

    float constexpr OD1_WHEEL_DIAMETER_MM = 50.0f; // Odom Wheel diameter in millimeters
    float constexpr OD2_WHEEL_DIAMETER_MM = 50.0f; // Odom Wheel diameter in millimeters

    float constexpr M_WHEEL_BASE_MM = 168.0f;    // Distance between the two motors wheels in millimeters
    float constexpr OD_WHEEL_BASE_MM = 200.0f;   // Distance between the two odom wheels in millimeters

    // MOTION CONFIGURATION

    float constexpr GO_MISSION_TOLERANCE_M = 0.005f;        // Tolerance for GO missions in meters
    float constexpr TURN_MISSION_TOLERANCE_RAD = 0.01f;     // Tolerance for TURN missions in radians

    float constexpr MAX_LINEAR_VELOCITY_M_S = 0.5f;    // Maximum linear velocity in metter per second
    float constexpr MAX_ANGULAR_VELOCITY_RAD_S = 3.5f;    // Maximum angular velocity in radians per second

    float constexpr LINEAR_ACCELERATION_M_S2 = 0.5f;      // Linear acceleration in metter per second squared
    float constexpr LINEAR_DECCELERATION_M_S2 = 0.3f;     // Linear deceleration in meters per second squared

    float constexpr ANGULAR_ACCELERATION_RAD_S2 = 3.0f;     // Angular acceleration in radians per second squared

    float constexpr LINEAR_JERK_M_S3 = 0.6f;              // Linear jerk (rate of change of acceleration) in m/s^3

    float constexpr LINEAR_EMERGENCY_MAX_DECELEATION = 3.0;
    float constexpr ANGULAR_EMERGENCY_MAX_DECELEATION = 9.0f;

    // SOFTWARE CONFIGURATION

    float constexpr CONTROL_LOOP_FREQUENCY_HZ = 25.0f; // Control loop frequency in Hertz
    float constexpr MOTOR_TIMER_FREQUENCY_HZ = 20000.0f; // Hardware timer frequency for motor stepping (Hz)
    float constexpr ODOM_MEASURE_FREQUENCY_HZ = 15.0f; // Odometry measurement frequency in Hertz
    float constexpr ODOM_FREQUENCY_HZ = 10.0f; // Control loop frequency in Hertz


    unsigned long int constexpr SERIAL_BAUDRATE = 115200; // Serial communication baudrate
    bool constexpr ENABLE_SERIAL_DEBUG = false; // Enable or disable serial debug messages

    int constexpr I2C_ADD = 0x30; // I2C address for the robot

} // namespace config

#endif // CONFIG_H
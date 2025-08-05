#ifndef CONFIG_H
#define CONFIG_H

#define RFID_TEST 1
#define BACK_FORTH 2
#define LIDAR_TEST 3
#define FORWARD_ONCE 4

// --- TUNING & CONFIGURATION CONSTANTS ---
const int MAX_RPM = 14500;            // Example: Maximum running RPM (TUNE)
const int MAX_REVERSE_RPM = -14500;   // Example: Maximum reverse RPM (TUNE)
const int SLOWDOWN_DISTANCE_CM = 200; // Distance to start braking (in cm)
const int STOP_DISTANCE_CM = 100;     // Desired final distance from gate (in cm)
const int HYSTERISIS_LOW_CM = 80;     // If closer than this, force stop (in cm)
const int HYSTERISIS_HIGH_CM = 120;   // If further than this (after stop), can restart (in cm)
const int RFID_EPC_OFFSET = 31;       // Offset to start readin EPC bytes

const float DESIRED_DECELERATION_MS2 = 1.5f; // Desired deceleration (m/s^2) - !! KEY TUNING PARAMETER !!
const float KP_BRAKE = 5.0f;                 // P-gain for braking - !! KEY TUNING PARAMETER !!
const float MAX_BRAKE_CURRENT_A = 50.0f;     // Max Amps for braking - SAFETY LIMIT
const float FINAL_BRAKE_CURRENT_A = 10.0f;   // Final brake current to hold position - SAFETY LIMIT
const float ERPM_TO_M_S_FACTOR = 0.0006374f; // !! VERIFY & CALCULATE THIS (WheelDia*PI)/(60*polepairs) !!
const float M_S_TO_RPM_FACTOR = 1.0f / ERPM_TO_M_S_FACTOR;
const float MPH_TO_RPM_FACTOR = 701.0f; // Conversion factor from MPH to RPM (1 MPH = 0.000277778 RPM)

const int LIDAR_ERROR_DIST_CM = 1200; // Distance to assume if Lidar errors (e.g., 12m)
const int MIN_ERPM_FOR_STOP = 50;     // RPM below which we consider it stopped

#endif // CONFIG_H
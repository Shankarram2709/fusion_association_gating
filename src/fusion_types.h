#ifndef FUSION_TYPES_H
#define FUSION_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// max definitions
static constexpr uint32_t MAX_TRACKS = 64;
static constexpr uint32_t MAX_MEASUREMENTS = 32;
static constexpr uint32_t STATE_DIM = 4;            // [x, y, vx, vy] (accx, accy, yaw, yawRate not defined for simplicity)
static constexpr uint32_t COV_DIM = 4;              // 4x4 Covariance Matrix

typedef float float32_t;

// --- Measurement Definitions ---

// Radar Raw Data (Sensor Frame)
struct MeasRadarRaw {
    float32_t range;       // Radial distance (m)
    float32_t azimuth;     // Horizontal Angle (rad)
    float32_t doppler;     // Radial Velocity (m/s)
    float32_t rcs;         // Cross Section (dBsm)
};

// Camera Raw Data (Sensor Frame)
struct MeasCameraRaw {
    float32_t u;           // center X (pixels)
    float32_t v;           // center Y (pixels)
    float32_t opt_flow_x;  // pixels/sec
    float32_t opt_flow_y;  // pixels/sec
    uint8_t class_id;      // class identifier
    float32_t confidence;  // detection score
};

// Bumper Frame
// Standardizes inputs
struct MeasConverted {
    float32_t x;           // longitudinal Pos (m)
    float32_t y;           // lateral Pos (m)
    float32_t vx;          // long. Velocity (m/s)
    float32_t vy;          // lat. Velocity (m/s)
    float32_t timestamp;   
    
    // Measurement Noise Variances
    float32_t var_x;
    float32_t var_y;
    float32_t var_vx;
    float32_t var_vy;
};

// Measurement Container
struct Measurement_t {
    uint32_t id;
    uint8_t sensor_id;      // 1=Radar, 2=Camera
    bool is_valid;          // Flag for processing status
    
    // Union allows storage of sensor-specific raw data efficiently
    union {
        MeasRadarRaw radar;
        MeasCameraRaw camera;
    } raw;

    MeasConverted converted; 
};

// --- Track Definition ---
struct Track_t {
    uint32_t id;
    bool is_active;
    
    // estimated state Vector [x, y, vx, vy]
    float32_t state[STATE_DIM]; 
    
    // estimation error covariance P
    float32_t P[STATE_DIM * STATE_DIM]; 

    // track management logic
    float32_t poe;          // Probability of Existence
    uint32_t age;           // Total cycles alive
    uint32_t missed_cycles; // Consecutive misses (for coasting)
    
    // Sensor Bitmask: 0x01=Radar, 0x02=Camera
    uint8_t sensor_mask;    
    
    // history Ring Buffer
    uint32_t history[5];    
    uint8_t hist_idx;
};

// Candidate for Data Association
struct AssocCandidate {
    int16_t track_idx;
    int16_t meas_idx;
    float32_t score;
};

#endif // FUSION_TYPES_H
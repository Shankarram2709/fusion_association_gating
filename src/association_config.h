#ifndef ASSOCIATION_CONFIG_H
#define ASSOCIATION_CONFIG_H

#include "fusion_types.h"

struct GatingParams {
    float32_t gate_x;          // Max longitudinal residual (m)
    float32_t gate_y;          // Max lateral residual (m)
    float32_t gate_vel_res;    // Max velocity vector diff magnitude (m/s)
    float32_t gate_range;      // Max radial distance diff (m)
    float32_t gate_azimuth;    // Max angular diff (rad)
    float32_t hybrid_thresh;   // Chi-Square 99% limit (4 DOF)
};

// Radar Config
static const GatingParams kRadarConfig = {
    .gate_x = 5.0f,
    .gate_y = 3.5f,
    .gate_vel_res = 2.5f,      
    .gate_range = 3.0f,
    .gate_azimuth = 0.15f,    
    .hybrid_thresh = 12.0f    
};

static const GatingParams kCameraConfig = {
    .gate_x = 8.0f,           
    .gate_y = 1.5f,            
    .gate_vel_res = 5.0f,      
    .gate_range = 8.0f,
    .gate_azimuth = 0.05f,     
    .hybrid_thresh = 13.27f
};

// --- Lifecycle Logic ---
#define POE_ACTIVE_THRESH   0.30f  // Track confirmed > 30%
#define POE_INC_MATCH       0.10f  // Increment on association
#define POE_DEC_MISS        0.05f  // Decay on miss
#define MAX_COAST_CYCLES    5      // Delete after 5 missed updates

#endif // ASSOCIATION_CONFIG_H
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
    5.0f,   // gate_x
    3.5f,   // gate_y
    2.5f,   // gate_vel_res
    3.0f,   // gate_range
    0.15f,  // gate_azimuth
    12.0f  // hybrid_thresh
};

static const GatingParams kCameraConfig = {
    8.0f,   // gate_x
    1.5f,   // gate_y
    5.0f,   // gate_vel_res
    8.0f,   // gate_range
    0.05f,  // gate_azimuth
    13.2f  // hybrid_thresh
};

// --- Lifecycle Logic ---
static constexpr float32_t POE_ACTIVE_THRESH = 0.30f;  // Track confirmed > 30%
static constexpr float32_t POE_INC_MATCH = 0.10f;  // Increment on association
static constexpr float32_t POE_DEC_MISS = 0.05f;  // Decay on miss
static constexpr uint32_t MAX_COAST_CYCLES = 5;      // Delete after 5 missed updates

#endif // ASSOCIATION_CONFIG_H
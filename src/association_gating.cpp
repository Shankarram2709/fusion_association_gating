#include "association_gating.h"
#include "association_math.h"
#include <math.h>

float32_t AssociationGating::computeCost(const Track_t* track, const Measurement_t* meas) {
    // Select sensor-specific gating parameters
    const GatingParams* params = (meas->sensor_id == 1) ? &kRadarConfig : &kCameraConfig;
    const MeasConverted* m_conv = &meas->converted;

    // --- Association gates ---
    // Cheaper checks first, followed by cost calculation

    if (!cartesianGate(track, m_conv, params)) return -1.0f;
    
    if (!velocityGating(track, m_conv, params)) return -1.0f;
    
    // convert states to polar to check geometric feasibility
    if (!polarGating(track, m_conv, params)) return -1.0f; 
    
    // ensure 1-to-1 sensor type mapping per track
    if (!gateBitmask(track, meas)) return -1.0f;

    // final Cost Calculation
    float32_t hybrid_cost = 0.0f;
    if (!hybridCost(track, m_conv, params, &hybrid_cost)) {
        return -1.0f; 
    }

    return hybrid_cost;
}

bool AssociationGating::cartesianGate(const Track_t* t, const MeasConverted* m, const GatingParams* p) {
    if (fabsf(t->state[0] - m->x) > p->gate_x) return false;
    if (fabsf(t->state[1] - m->y) > p->gate_y) return false;
    return true;
}

bool AssociationGating::velocityGating(const Track_t* t, const MeasConverted* m, const GatingParams* p) {
    // Check the magnitude of the velocity difference vector
    float32_t diff = AssociationMath::vectorDiffMag(t->state[2], t->state[3], m->vx, m->vy);
    if (diff > p->gate_vel_res) return false;
    return true;
}

bool AssociationGating::polarGating(const Track_t* t, const MeasConverted* m, const GatingParams* p) {
    float32_t t_r, t_az, m_r, m_az;
    
    // Transform both Track and Meas (Bumper Frame) to Polar
    AssociationMath::cartesianToPolar(t->state[0], t->state[1], &t_r, &t_az);
    AssociationMath::cartesianToPolar(m->x, m->y, &m_r, &m_az);

    if (fabsf(t_r - m_r) > p->gate_range) return false;
    
    float32_t az_diff = fabsf(AssociationMath::normalizeAngle(t_az - m_az));
    if (az_diff > p->gate_azimuth) return false;
    
    return true;
}

bool AssociationGating::gateBitmask(const Track_t* t, const Measurement_t* m) {
    uint8_t bit = (m->sensor_id == 1) ? 0x01 : 0x02;
    // If bit is already set, this sensor type is already occupied for this track
    if ((t->sensor_mask & bit) != 0) return false;
    return true;
}

bool AssociationGating::hybridCost(const Track_t* t, const MeasConverted* m, const GatingParams* p, float32_t* out_cost) {
    // Innovation vector y = z - Hx
    float32_t y_tilde[4] = {
        m->x - t->state[0], 
        m->y - t->state[1], 
        m->vx - t->state[2], 
        m->vy - t->state[3]
    };

    // Construct Innovation Covariance S = P + R (Assuming H=I)
    float32_t S[16];
    for(int32_t i = 0; i < 16; i++) S[i] = t->P[i];
    
    // Add Measurement Noise R to diagonal
    S[0] += m->var_x; 
    S[5] += m->var_y; 
    S[10] += m->var_vx; 
    S[15] += m->var_vy;

    // Perform Cholesky Decomposition for numerical stability check
    float32_t L[16] = {0};
    if (!AssociationMath::choleskyDecomposition(S, L)) {
        return false; // reject update
    }

    // Compute Mahalanobis Distance: d^2 = y^T * S^-1 * y
    // diagonal approximation
    float32_t mahalanobis = (y_tilde[0] * y_tilde[0]) / S[0]; 

    // Log Determinant Penalty: ln(|S|) = 2 * sum(ln(L_ii))
    float32_t log_det = 0.0f;
    for(int32_t i = 0; i < 4; i++) {
        log_det += 2.0f * logf(L[(i * 4) + i]);
    }

    *out_cost = mahalanobis + log_det;
    
    if (*out_cost > p->hybrid_thresh) return false;
    return true;
}
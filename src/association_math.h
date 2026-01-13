#ifndef ASSOCIATION_MATH_H
#define ASSOCIATION_MATH_H

#include "fusion_types.h"
#include <math.h>

static constexpr float32_t PI = 3.14159265f;
// conceptual Sensor Mount Offsets (from rear axle center)
static constexpr float32_t RADAR_DX = 3.8f;
static constexpr float32_t CAM_DX = 1.5f;

class AssociationMath {
public:
    // Wraps angle to [-PI, PI]
    static float32_t normalizeAngle(float32_t angle) {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    static void cartesianToPolar(float32_t x, float32_t y, float32_t* r, float32_t* az) {
        *r = sqrtf((x * x) + (y * y));
        *az = atan2f(y, x);
    }

    // Calculates the Euclidean magnitude of the difference vector
    // |v1 - v2| = sqrt((vx1-vx2)^2 + (vy1-vy2)^2)
    static float32_t vectorDiffMag(float32_t vx1, float32_t vy1, float32_t vx2, float32_t vy2) {
        float32_t dx = vx1 - vx2;
        float32_t dy = vy1 - vy2;
        return sqrtf((dx * dx) + (dy * dy));
    }

    // Sensor Frame -> Vehicle Bumper Frame
    // Currently assumes 0 yaw mounting, translation only
    static void transformToVehicle(float32_t sx, float32_t sy, float32_t dx_mount, MeasConverted* out) {
        out->x = sx + dx_mount;
        out->y = sy; 
        // Rotation matrix not implemented
    }

    // Manual Cholesky Decomposition (L * L^T = S)
    // Validates that the Innovation Covariance S is Positive Definite.
    // Returns false if decomposition fails (numerical instability).
    static bool choleskyDecomposition(const float32_t S[16], float32_t L[16]) {
        for (int32_t i = 0; i < 4; i++) {
            for (int32_t j = 0; j <= i; j++) {
                float32_t sum = 0.0f;
                for (int32_t k = 0; k < j; k++) {
                    sum += L[(i * 4) + k] * L[(j * 4) + k];
                }
                
                if (i == j) {
                    float32_t val = S[(i * 4) + i] - sum;
                    if (val <= 0.0f) return false; // Matrix not PD
                    L[(i * 4) + j] = sqrtf(val);
                } else {
                    L[(i * 4) + j] = (1.0f / L[(j * 4) + j]) * (S[(i * 4) + j] - sum);
                }
            }
        }
        return true;
    }
};

#endif // ASSOCIATION_MATH_H
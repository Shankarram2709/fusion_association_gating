#ifndef ASSOCIATION_GATING_H
#define ASSOCIATION_GATING_H

#include "fusion_types.h"
#include "association_config.h"

class FusionGating {
public:
    // Main Pipeline: Computes association cost
    // Returns -1.0f if any gate rejects the pair (Gated).
    static float32_t computeCost(const Track_t* track, const Measurement_t* meas);

private:
    // Positional- based gating
    static bool cartesianGate(const Track_t* t, const MeasConverted* m, const GatingParams* p);
    
    // velocity gating
    static bool velocityGating(const Track_t* t, const MeasConverted* m, const GatingParams* p);
    
    // range and azimuth based gate
    static bool polarGating(const Track_t* t, const MeasConverted* m, const GatingParams* p);
    
    // sensor bitmask
    static bool gateBitmask(const Track_t* t, const Measurement_t* m);
    
    // hybrid Cost (Mahalanobis + LogDet)
    static bool hybridCost(const Track_t* t, const MeasConverted* m, const GatingParams* p, float32_t* out_cost);
};

#endif // ASSOCIATION_GATING_H
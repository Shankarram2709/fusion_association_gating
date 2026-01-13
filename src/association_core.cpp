#include "fusion_types.h"
#include "association_gating.h"
#include "association_math.h"
#include <stdio.h>

// allocations of tracks and candidates
static Track_t g_tracks[MAX_TRACKS];
static constexpr uint32_t MAX_CANDIDATES = MAX_TRACKS * MAX_MEASUREMENTS;
static AssocCandidate g_candidates[MAX_CANDIDATES];

// helper for Greedy algo
void sortCandidates(AssocCandidate* arr, int32_t count) {
    for (int32_t i = 0; i < count - 1; i++) {
        for (int32_t j = 0; j < count - i - 1; j++) {
            if (arr[j].score > arr[j + 1].score) { // Ascending order
                AssocCandidate temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

// Main Processing Cycle
void associationProcessCycle(Measurement_t* meas_list, int32_t num_meas) {
    
    // Pre-Processing: Transform Raw Sensor Data to Bumper Frame
    for(int32_t i = 0; i < num_meas; i++) {
        float32_t mount_dx = (meas_list[i].sensor_id == 1) ? RADAR_DX : CAM_DX;
        
        if (meas_list[i].sensor_id == 1) {
            // Radar: Polar -> Cartesian -> Transform
            float32_t r = meas_list[i].raw.radar.range;
            float32_t az = meas_list[i].raw.radar.azimuth;
            float32_t lx = r * cosf(az);
            float32_t ly = r * sinf(az);
            AssociationMath::transformToVehicle(lx, ly, mount_dx, &meas_list[i].converted);
        } else {
            // Camera: Pixels Transform
             AssociationMath::transformToVehicle(meas_list[i].raw.camera.u, 
                                            meas_list[i].raw.camera.v, 
                                            mount_dx, &meas_list[i].converted);
        }
    }

    // 2. Gating & Scoring
    int32_t candidate_count = 0;
    for (int32_t t = 0; t < MAX_TRACKS; t++) {
        if (!g_tracks[t].is_active) continue;

        for (int32_t m = 0; m < num_meas; m++) {
            if (candidate_count >= MAX_CANDIDATES) break;
            float32_t cost = AssociationGating::computeCost(&g_tracks[t], &meas_list[m]);
            
            if (cost >= 0.0f) {
                g_candidates[candidate_count].track_idx = (int16_t)t;
                g_candidates[candidate_count].meas_idx = (int16_t)m;
                g_candidates[candidate_count].score = cost;
                candidate_count++;
            }
        }
        if (candidate_count >= MAX_CANDIDATES) break;
    }

    // 3. Global Association (Sorted Greedy)
    sortCandidates(g_candidates, candidate_count);

    for (int32_t i = 0; i < candidate_count; i++) {
        int32_t t_idx = g_candidates[i].track_idx;
        int32_t m_idx = g_candidates[i].meas_idx;
        Track_t* trk = &g_tracks[t_idx];
        Measurement_t* meas = &meas_list[m_idx];

        if (meas->is_valid) { 
            meas->is_valid = false; // Mark measurement as consumed
            
            // Update Track Metadata
            uint8_t sensor_bit = (meas->sensor_id == 1) ? 0x01 : 0x02;
            trk->sensor_mask |= sensor_bit;
            
            trk->poe += POE_INC_MATCH;
            if (trk->poe > 1.0f) trk->poe = 1.0f;
            
            trk->history[trk->hist_idx] = meas->id;
            trk->hist_idx = (trk->hist_idx + 1) % 5;
            
            // Kalman Filter Update step would be called here
        }
    }
}
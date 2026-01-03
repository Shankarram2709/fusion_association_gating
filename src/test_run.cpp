#include "fusion_types.h"
#include <stdio.h>

void associationProcessCycle(Measurement_t* meas_list, int32_t num_meas);

int main() {
    printf("Initializing association module...\n");

    // Create a mock Radar object
    Measurement_t meas_list[1];
    meas_list[0].id = 101;
    meas_list[0].sensor_id = 1; // Radar
    meas_list[0].is_valid = true;
    meas_list[0].raw.radar.range = 15.0f;
    meas_list[0].raw.radar.azimuth = 0.05f; 
    meas_list[0].raw.radar.doppler = 22.0f;
    
    // Set mock noise params
    meas_list[0].converted.var_x = 0.5f;
    meas_list[0].converted.var_y = 0.2f;

    // Execute Cycle
    associationProcessCycle(meas_list, 1);

    printf("Cycle Complete. No tracks to associate yet.\n");
    return 0;
}
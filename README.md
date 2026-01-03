# Sensor Association & Gating Module

## Overview
Implements a conceptual **Data Association Module** for ADAS Sensor Fusion (Radar + Camera).
It is designed adhering to **C++14** and **static memory allocation** (no STL/Heap).

## Key Features
1.  **Gating Strategy**: 
    - applies cheap gates first (Cartesian -> Velocity).
    - **Hybrid Cost** (Mahalanobis + LogDet) only for valid candidates.
2.  **Generic "Polar" Gating**:
    - Converts Track State (Cartesian) to Polar to validate against Raw Sensor data (e.g., Radar Range/Azimuth)
3.  **Sensor Logic**:
    - Uses a `MeasConverted` structure (Vehicle Bumper Frame) for generic fusion logic.
    - Preserves `MeasRaw` (Sensor Frame) only for specific transformations.
4.  **Bitmask Logic**:
    - Enforces a mutual "1 Radar + 1 Camera" per track rule using bitwise operations 
5.  **Numerical Stability**: via cholesky decomposition

## Project Structure
- `src/association_core.cpp`: Main runtime loop
- `src/association_gating.cpp`: The Gating logic
- `src/association_math.h`: Math utils
- `src/association_config.h`: Tunable parameters
- `src/fusion_types.h`: Standard structs for Tracks and Measurements.

```bash
g++ -std=c++14 src/association_core.cpp src/association_gating.cpp src/test_run.cpp -o association_test -Isrc
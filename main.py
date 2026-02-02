"""
Main entry point for PathPlanner.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, '..')
sys.path.insert(0, str(Path(__file__).parent.parent))

from PathPlanner.core.planners import run_path_planner
from tests.test_cases import test_case_graz_4uavs, RESULTS_DIR, mission_results_dir

def main():
    # Use environment variable for QGroundControl Missions directory, with a default relative path
    default_missions_dir = Path(__file__).parent / "missions"  # Relative to PathPlanner/
    mission_results_dir = Path(os.getenv("QGC_MISSIONS_DIR", str(default_missions_dir)))
    mission_results_dir.mkdir(parents=True, exist_ok=True)
    
    # ============================================================================
    # DEFINE DEFAULT TEST CASE Inputs (Generic, No Real GPS Data)
    # ============================================================================
    
    # Number of UAVs (default: 1 for simple test)
    num_uavs = 1
    
    # Initial UAV positions (default: single UAV at center of test area)
    init_pos_uavs_gps = [(0.5, 0.5)]  # Placeholder GPS-like coordinates
    
    # Polygon GPS coordinates (default: simple rectangle)
    poly_gps = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]  # Placeholder rectangle
    
    # Hole GPS coordinates (default: no holes)
    holes_gps = []  # Empty for no holes
    
    # Terrain altitude and mission AGL (default values)
    terrain_alt = 0  # in meters (sea level default)
    mission_agl = 10  # in meters (low altitude for test)
    
    # Lane width and step size (default values)
    lane_width = 10  # meters
    step_size = 10   # meters
    
    # Run the planner with defaults
    run_path_planner(default_missions_dir, mission_results_dir, poly_gps, holes_gps, init_pos_uavs_gps, terrain_alt, mission_agl, lane_width, step_size, num_uavs)
    
    
    # Use a specific test case
    # config = test_case_graz_4uavs
    # run_path_planner(RESULTS_DIR, mission_results_dir, **config)

if __name__ == "__main__":
    main()
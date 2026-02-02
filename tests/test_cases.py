# test_cases.py
# This file contains test configurations for the PathPlanner.
# Each test case is a dictionary with the required inputs for run_path_planner.

from pathlib import Path

# ============================================================================
# TEST CASE: Stubaier Gletscher Avalanche Area
# ============================================================================
test_case_stubaier = {
    'num_uavs': 1,
    'init_pos_uavs_gps': [(46.990349, 11.091969)],  # UAV initial position
    'poly_gps': [(46.988522, 11.099439), (46.992095, 11.101484),
                 (46.990902, 11.094667), (46.988461, 11.093070)],  # Polygon coordinates
    'holes_gps': [],  # No holes (changed from hole_gps to holes_gps)
    'terrain_alt': 360,  # meters MSL
    'mission_agl': 25,   # meters AGL
    'lane_width': 15,    # meters
    'step_size': 15      # meters
}

# ============================================================================
# TEST CASE: Cologne Area (Example with 2 UAVs)
# ============================================================================
test_case_cologne_2uavs = {
    'num_uavs': 2,
    'init_pos_uavs_gps': [(50.939387, 7.476801), (50.938941, 7.480121)],
    'poly_gps': [(50.939486, 7.477263), (50.940835, 7.477050), (50.941526, 7.480789), 
                 (50.940672, 7.481751), (50.940217, 7.480247), (50.939789, 7.480415)],
    'holes_gps': [(50.940362, 7.477996), (50.940265, 7.479147)],  # Hole inside (changed from hole_gps to holes_gps)
    'terrain_alt': 50,   # meters MSL (example)
    'mission_agl': 20,   # meters AGL
    'lane_width': 10,    # meters
    'step_size': 10      # meters
}

# ============================================================================
# TEST CASE: Graz Area (Example with 4 UAVs)
# ============================================================================
test_case_graz_4uavs = {
    'num_uavs': 4,
    'init_pos_uavs_gps': [(47.045870, 15.386828), (47.045528, 15.386987), 
                          (47.044591, 15.389125), (47.045226, 15.387547)],
    'poly_gps': [(47.045528, 15.386987), (47.044591, 15.389125)],  # Rectangle diagonal
    'holes_gps': [(47.045226, 15.387547), (47.045108, 15.388418)],  # Hole diagonal (changed from hole_gps to holes_gps)
    'terrain_alt': 350,  # meters MSL
    'mission_agl': 30,   # meters AGL
    'lane_width': 5,    # meters
    'step_size': 5      # meters
}

# ============================================================================
# PERSONAL PATHS (for your setup only - do not commit to repo)
# ============================================================================
# These are specific to your machine; use environment variables or keep local.
GROUNDCONTROL_ROOT = Path(__file__).parent.parent.parent  # Navigate to GroundControl root
RESULTS_DIR = GROUNDCONTROL_ROOT / "results" / "PathPlanner_04"/ "test_case_graz_4uavs"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

# Export to QGroundControl Missions Directory (your personal path)
mission_results_dir = Path("/mnt/c/Users/m-due/Documents/QGroundControl/Missions/PathPlanner_04/test_case_graz_4uavs")
mission_results_dir.mkdir(parents=True, exist_ok=True)

# ============================================================================
# USAGE EXAMPLE
# ============================================================================
# In main.py or a test script:
# from test_cases import test_case_stubaier, RESULTS_DIR, mission_results_dir
# config = test_case_stubaier
# run_path_planner(RESULTS_DIR, mission_results_dir, **config)
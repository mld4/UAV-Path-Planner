"""
Input/Output: Mission export and visualization.
"""

import json
from PathPlanner.utils.conversions import convert_enu_to_gps


def save_mission_to_qgc(uav_data, mission_agl, filename, results_dir):
    """
    Purpose
    -------
    Save UAV path as QGroundControl .plan file.
    
    Inputs
    ------
    uav_data : Dict
        UAV GPS data.
    mission_agl : float
        Mission altitude.
    filename : str
        Output filename.
    results_dir : Path
        Output directory.
    
    Outputs
    -------
    mission_path : Path
        Path to saved file.
    
    Notes
    -----
    - Includes HOME, TAKEOFF, waypoints, and RTL.
    """
    waypoints_gps = uav_data['path_points_gps']
    init_pos_uav_gps = uav_data['init_pos_gps']
    init_alt = uav_data['init_alt']
    
    print(f"\n{'='*80}")
    print(f"CREATING QGROUNDCONTROL MISSION FILE FOR UAV {uav_data['uav_idx']}")
    print(f"{'='*80}")
    print(f"Initial Altitude: {init_alt}m")
    print(f"Mission Altitude (AGL): {mission_agl}m")
    print(f"Total Waypoints: {len(waypoints_gps)}")
    
    # ============================================================================
    # STEP 1: Create Mission Structure
    # ============================================================================
    qgc_mission = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "version": 1,
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {
            "cruiseSpeed": 6,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": [],
            "plannedHomePosition": [
                float(init_pos_uav_gps[0]),
                float(init_pos_uav_gps[1]),
                float(init_alt)  
            ],
            "vehicleType": 2,
            "version": 2
        }
    }
    
    # ============================================================================
    # STEP 2: Add HOME Waypoint (Index 0)
    # ============================================================================
    home_waypoint = {
        "AMSLAltAboveTerrain": None,
        "Altitude": float(init_alt),  # Home at initial altitude
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 16,
        "doJumpId": 1,
        "frame": 3,
        "params": [
            0, 
            0, 
            0, 
            None, 
            float(init_pos_uav_gps[0]), 
            float(init_pos_uav_gps[1]), 
            float(init_alt)
            ],
        "type": "SimpleItem"
    }
    qgc_mission["mission"]["items"].append(home_waypoint)
    
    # ============================================================================
    # STEP 3: Add TAKEOFF Command (Index 1)
    # ============================================================================
    takeoff_waypoint = {
        "AMSLAltAboveTerrain": None,
        "Altitude": float(init_alt),
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 22,  # MAV_CMD_NAV_TAKEOFF
        "doJumpId": 2,
        "frame": 3,
        "params": [
            0,                      # Pitch angle (0 = level)
            0,                      # Empty
            0,                      # Empty
            None,                   # Yaw angle (NaN = no change)
            float(init_pos_uav_gps[0]),   # Latitude
            float(init_pos_uav_gps[1]),   # Longitude
            float(init_alt)               # Takeoff altitude
        ],
        "type": "SimpleItem"
    }
    qgc_mission["mission"]["items"].append(takeoff_waypoint)
    
    # ============================================================================
    # STEP 4: Add All Path Waypoints (at mission_agl)
    # ============================================================================
    for i, (lat, lon, alt) in enumerate(waypoints_gps):
        waypoint = {
            "AMSLAltAboveTerrain": None,
            "Altitude": mission_agl,  # Path waypoints at mission altitude
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,
            "doJumpId": i + 3,  # Offset by 3 (HOME + TAKEOFF)
            "frame": 3,
            "params": [
                0, 
                0.2, 
                0, 
                None, 
                float(lat), 
                float(lon), 
                float(mission_agl)
            ],
            "type": "SimpleItem"
        }
        qgc_mission["mission"]["items"].append(waypoint)
   
    # ============================================================================
    # STEP 5: Add RETURN TO LAUNCH Command
    # ============================================================================
    rth_waypoint = {
        "autoContinue": True,
        "command": 20,
        "doJumpId": len(waypoints_gps) + 3,
        "frame": 2,
        "params": [0, 0, 0, 0, 0, 0, 0],
        "type": "SimpleItem"
    }
    qgc_mission["mission"]["items"].append(rth_waypoint)
    
    # ============================================================================
    # STEP 6: Save Mission File
    # ============================================================================
    mission_path = results_dir / filename
    with open(mission_path, 'w') as f:
        json.dump(qgc_mission, f, indent=2)
    
    print(f"\n✓ Mission file saved: {mission_path}")
    print(f"  - Total items: {len(qgc_mission['mission']['items'])}")
    print(f"  - Sequence:")
    print(f"    1. HOME (alt: {init_alt}m)")
    print(f"    2. TAKEOFF (alt: {init_alt}m)")
    print(f"    3. Waypoints 1-{len(waypoints_gps)} (alt: {mission_agl}m)")
    print(f"    4. RETURN TO LAUNCH")
    print(f"{'='*80}\n")
    
    return mission_path

def convert_in_gps_export(uav_data_ls, reference, init_pos_uavs_gps, mission_agl, RESULTS_DIR, mission_results_dir):
    '''
    Purpose
    -------
    Convert UAV paths to GPS and export QGroundControl missions.
    
    Inputs
    ------
    uav_data_ls : List[Dict]
        UAV data in ENU.
    reference : Tuple[float, float, float]
        Reference GPS point.
    init_pos_uavs_gps : List[Tuple[float, float]]
        UAV GPS positions.
    mission_agl : float
        Mission altitude.
    RESULTS_DIR : Path
        Plot directory.
    mission_results_dir : Path
        Mission directory.
    
    Outputs
    -------
    uav_data_gps_ls : List[Dict]
        UAV data in GPS.
    
    Notes
    -----
    - Saves mission files to mission_results_dir
    '''
    # ============================================================================
    #  Convert Waypoints in GPS Data for each UAV
    # ============================================================================
    uav_data_gps_ls = []
    for uav_idx in range(len(uav_data_ls)):
        # Find the existing UAV data entry/dictionary
        for uav_data in uav_data_ls:
            if uav_data['uav_idx'] == uav_idx:
                
                waypoints_gps = []
                # --- Convert Waypoints to GPS ---
                for waypoint in uav_data['path_points']:
                    waypoint_3d = (waypoint[0], waypoint[1], mission_agl)
                    lat, lon, alt = convert_enu_to_gps(waypoint_3d, reference)
                    waypoints_gps.append((lat, lon, alt))
                start_end_points_gps = []
                for start_pt, end_pt in uav_data['start_end_points']:
                    start_pt_3d = (start_pt[0], start_pt[1], mission_agl)
                    end_pt_3d = (end_pt[0], end_pt[1], mission_agl)
                    start_gps = convert_enu_to_gps(start_pt_3d, reference)
                    end_gps   = convert_enu_to_gps(end_pt_3d,   reference)
                    start_end_points_gps.append((start_gps, end_gps))

                uav_data_gps_ls.append({
                    'uav_idx': uav_data['uav_idx'],
                    'init_pos_gps': init_pos_uavs_gps[uav_data['uav_idx']],
                    'init_alt': uav_data['init_alt'],
                    'path_points_gps': waypoints_gps,
                    'start_end_points_gps': start_end_points_gps,
                })
                # --- Save Mission in QGroundControl Format ---        
                filename = f"UAV{uav_data['uav_idx']}_mission.plan"              
                save_mission_to_qgc(
                    uav_data_gps_ls[-1],
                    mission_agl,
                    filename=filename,
                    results_dir=mission_results_dir  
                    )
                break  
    return uav_data_gps_ls
"""
Core planners for UAV path planning.
"""

import numpy as np
from pathlib import Path
from shapely.geometry import Polygon
from PathPlanner.core.geometry import scan_with_inset, Get_Start_End_Points, get_decomposed_area
from PathPlanner.io.mission_export import convert_in_gps_export
from PathPlanner.io.visualization import plot_results, plot_assignment_results
from PathPlanner.utils.conversions import convert_search_area_polygon, convert_gps_point_to_enu
from PathPlanner.core.assignment import task_assign_jv

def one_uav_planner(num_uavs, init_pos_uav_enu_ls, init_pos_uavs_gps, reference, polygon, cells, corner_points_ls, lane_width, step_size, terrain_alt, mission_agl, RESULTS_DIR, mission_results_dir):
    '''
    Purpose
    -------
    Generate a lawnmower scan path within a rectangular cell, starting at specified points if signal_special_scan is True.
    The cell should be pre-inset/shrunken to account for UAV size. For special scans, determines scan direction based on first_point and second_point,
    and may start at second_point if conditions are met. Alternates row directions for efficient coverage.

    Inputs
    ------
    cell : shapely.geometry.Polygon
        The decomposed cell to scan (should be pre-inset).
    lane_width : float
        The spacing between adjacent flight lines.
    step_size : float
        The spacing between waypoints along each segment.
    first_point : tuple[float, float] or None
        First corner point for scan direction determination.
    second_point : tuple[float, float] or None
        Second corner point for scan direction determination.
    end_points : list[tuple[float, float]] or None
        List of remaining corner points for direction logic.
    signal_special_scan : bool
        If True, use first_point/second_point for starting logic; otherwise, use default scanning.

    Outputs
    -------
    path_points : list[tuple[float, float]]
        Flat list of (x, y) waypoints ordered as flown: row-by-row, alternating directions.
    path_dist : float
        Total distance of the path.

    Notes
    -----
    - For special scans: Determines scan axis (x or y) based on dx/dy, sets direction based on start_point position,
      and may start at second_point with reversed direction if conditions are met.
    - Alternates direction (-1 or 1) for each row to create a back-and-forth pattern.
    - Samples points every step_size along each intersecting segment, including exact start/end points.
    - Returns empty lists if the cell is invalid or no intersections are found.
    - Path distance is calculated as the sum of Euclidean distances between consecutive waypoints.
    '''
    uav_idx = 0
    init_pos_uav = init_pos_uav_enu_ls[uav_idx][:2]
    # --- Get Start and End Points for the First Cell ---
    first_cell_idx, first_point, _, _, second_point, end_points= Get_Start_End_Points(corner_points_ls, init_pos_uav, signal_first_cell=True)
    print(f"Cell {first_cell_idx} is the first Cell")
    print(f"The First Point is: {first_point}")
    print(f"The Second Point is: {second_point}")
    print(f"The End Points are: {end_points}")

    # --- Generate Path for First Cell ---
    path = []
    num_added_wp_cell = []
    path_points, path_dist = scan_with_inset(cells[first_cell_idx], lane_width, step_size, first_point, second_point, end_points, signal_special_scan=True)
    path.extend(path_points)
    num_added_wp_cell.append(len(path_points))
    start_end_points_ls = [(path_points[0], path_points[-1])]
    prev_cell_idx_ls = [first_cell_idx]

    # --- Generate Path for Remaining Cells ---
    for i, cell in enumerate(cells):
        if i == first_cell_idx:
            continue  # Skip first cell (already processed)
        # --- Get Start and End Points for the Next Cell ----
        next_cell_idx, first_point, _, first_dist, second_point, end_points_ls = Get_Start_End_Points(
            corner_points_ls, init_pos_uav, signal_first_cell=False, 
            prev_cell_idx_ls=prev_cell_idx_ls, 
            end_point_prev=path_points[-1])

        # --- Generate Path for the Next Cell ---
        path_points, path_dist = scan_with_inset(cells[next_cell_idx], lane_width, step_size, first_point, second_point, end_points_ls, signal_special_scan=True) 

        path.extend(path_points)
        num_added_wp_cell.append(len(path_points))
        prev_cell_idx_ls.append(next_cell_idx) # Update for next iteriation
        start_end_points_ls.append((path_points[0], path_points[-1]))  # Track all start and end points
    # print(f"Start and End Points for each Cell: {start_end_points_ls}")
    uav_data_ls = []
    uav_data_ls.append({
            'uav_idx': uav_idx,
            'init_pos': init_pos_uav_enu_ls[uav_idx],
            'path_points': path,
            'start_end_points': start_end_points_ls,
            'init_alt': mission_agl 
            })

    # ============================================================================
    #  Convert Waypoints in GPS Data for each UAV
    # ============================================================================
    uav_data_gps_ls = convert_in_gps_export(uav_data_ls, reference, init_pos_uavs_gps, mission_agl, 
                                            RESULTS_DIR,  mission_results_dir)
    
    # --- PLOT RESULTS ---
    plot_results(polygon, lane_width, cells=cells, uav_data_ls=uav_data_ls, results_dir=RESULTS_DIR,
                corner_points_ls=corner_points_ls)
    return uav_data_ls, uav_data_gps_ls


def multiple_uav_planner(num_uavs, init_pos_uav_enu_ls, init_pos_uavs_gps, reference, polygon, cells, corner_points_ls, lane_width, step_size, terrain_alt, mission_agl, RESULTS_DIR, mission_results_dir):
    '''
    Purpose
    -------
    Assign cells to UAVs using cost-based optimization and plan parallel paths for equal UAV/cell count.
    
    Strategy
    -------
    - Compute cost matrix based on path distance and initial travel distance.
    - Use Hungarian algorithm for optimal UAV-cell assignment.
    - Assign altitudes based on cost (higher cost = higher altitude).
    - Generate paths, convert to GPS, and export missions.
    
    Inputs
    ------
    num_uavs : int
        Number of UAVs (must equal number of cells).
    init_pos_uav_enu_ls : List[Tuple[float, float, float]]
        UAV initial positions in ENU.
    init_pos_uavs_gps : List[Tuple[float, float]]
        UAV initial positions in GPS.
    reference : Tuple[float, float, float]
        Reference GPS point.
    polygon : shapely.geometry.Polygon
        Original search area.
    cells : List[Polygon]
        Decomposed cells.
    corner_points_ls : List[List[Tuple[float, float]]]
        Corner points of each cell.
    lane_width : float
        Spacing between scan lines.
    step_size : float
        Spacing between waypoints.
    terrain_alt : float
        Ground elevation (MSL).
    mission_agl : float
        Flight altitude above ground.
    RESULTS_DIR : Path
        Directory for debug plots.
    mission_results_dir : Path
        Directory for QGroundControl missions.
    
    Outputs
    -------
    uav_data_ls : List[Dict]
        UAV path data in ENU.
    uav_data_gps_ls : List[Dict]
        UAV path data in GPS.
    
    Notes
    -----
    - Requires num_uavs == len(cells).
    - Saves assignment plots, path plots, and mission files
    '''
    # Initialize Cost Matrix: rows = UAVs, columns = jobs/cells
    num_jobs = len(cells)
    cost_matrix = np.zeros((num_uavs, num_jobs))

    # --- GENERATE COST MATRIX FOR ALL UAV-CELL PAIRS ---
    # Store all path data temporarily
    all_uav_cell_data = {}  # key: (uav_idx, cell_idx), value: {path_points, start_end_points, ...}
    for m, init_pos_uav in enumerate(init_pos_uav_enu_ls):
        init_pos_uav = init_pos_uav[:2]
        # print(f"\n{'='*40}\nUAV {m} Initial Position: {init_pos_uav}\n{'='*40}")
        for i, cell in enumerate(cells):
            # print(f"\nCell: {i}")
            _, first_point, _, init_dist, second_point, end_points= Get_Start_End_Points([corner_points_ls[i]], init_pos_uav, signal_first_cell=True)
            
            path_points, path_dist = scan_with_inset(cell, lane_width, step_size, first_point, second_point, end_points, signal_special_scan=True)
            # print(f"UAV {m}, Cell {i} → Path Distance: {path_dist:.2f}m, Init Dist: {init_dist:.2f}m")
            cost_value = path_dist + (init_dist * 100)  # Weighting factor of 100 for init_dist
            cost_matrix[m, i] = cost_value
            
            # Store path data for this UAV-Cell pair
            all_uav_cell_data[(m, i)] = {
                'path_points': path_points,
                'start_end_points': [(path_points[0], path_points[-1])]
            }
        # print(f"\n{'='*40}")

    # --- Assigned Jobs JV ---
    row_ind, col_ind = task_assign_jv(cost_matrix)
    # print(f"Cost Matrix:\n{cost_matrix}")
    # --- Store Data for each UAV
    uav_data_ls = []
    task_assign_ls = []

    for uav_idx, cell_idx in zip(row_ind, col_ind):
        cost_val = cost_matrix[uav_idx, cell_idx]

        path_data = all_uav_cell_data[(uav_idx, cell_idx)]

        uav_data_ls.append({
            'uav_idx': uav_idx,
            'init_pos': init_pos_uav_enu_ls[uav_idx],
            'path_points': path_data['path_points'],
            'start_end_points': path_data['start_end_points']
        })

        task_assign_ls.append((uav_idx, cell_idx, cost_val))

        # print(f"Cost Matrix Entry: UAV {uav_idx}, Cell {cell_idx} → Cost: {cost_val:.2f}")
        # print(f"Start and End Points of UAV {uav_idx}, Cell {cell_idx}: {path_data['start_end_points']}")
    
    
    # --- PLOT RESULTS ---
    plot_results(polygon, lane_width, cells=cells, uav_data_ls=uav_data_ls, results_dir=RESULTS_DIR,
                corner_points_ls=corner_points_ls)
    plot_assignment_results(task_assign_ls, results_dir=RESULTS_DIR)

    # --- different Init Altitudes based on assigned Cost ---
    # Sort assignments by cost ascending
    cost_sorted = sorted(task_assign_ls, key=lambda x: x[2], reverse=True)  # Sort by cost value
    
    init_alt = sorted([mission_agl + (i+1) * ((-1) ** i) for i in range(num_uavs)]) # for even i -> positive & for odd i -> negative
    # print(f"init_alt{init_alt}")
 
    # add the Initial Altitude to uav data
    for cost_idx, (uav_idx, cell_idx, cost) in enumerate(cost_sorted):
        for uav_data in uav_data_ls:
            if uav_data['uav_idx'] == uav_idx:
                uav_data['init_alt'] = init_alt[cost_idx]
                # print(f"UAV {uav_idx} → Cost: {cost:.2f}, Init Altitude: {init_alt[cost_idx]:.1f}m")

    # ============================================================================
    #  Convert Waypoints in GPS Data for each UAV
    # ============================================================================
    uav_data_gps_ls = convert_in_gps_export(uav_data_ls, reference, init_pos_uavs_gps, mission_agl, 
                                            RESULTS_DIR, mission_results_dir)
    
    return uav_data_ls, uav_data_gps_ls


def too_many_uav_planner(num_uavs, init_pos_uav_enu_ls, init_pos_uavs_gps, reference, polygon, cells, corner_points_ls, lane_width, step_size, terrain_alt, mission_agl, RESULTS_DIR, mission_results_dir):
    '''
    Purpose
    -------
    Handle cases with more UAVs than cells by excluding the UAV with the highest total cost and re-planning with the remaining UAVs.
    
    Strategy
    -------
    - Compute a cost matrix for all UAV-cell pairs.
    - Calculate total cost per UAV and exclude the one with the highest cost.
    - Re-run planning with 1, 2, or 4 UAVs based on the remaining count.
    
    Inputs
    ------
    num_uavs : int
        Number of UAVs (greater than number of cells).
    init_pos_uav_enu_ls : List[Tuple[float, float, float]]
        UAV initial positions in ENU.
    init_pos_uavs_gps : List[Tuple[float, float]]
        UAV initial positions in GPS.
    reference : Tuple[float, float, float]
        Reference GPS point.
    polygon : shapely.geometry.Polygon
        Original search area.
    cells : List[Polygon]
        Decomposed cells.
    corner_points_ls : List[List[Tuple[float, float]]]
        Corner points of each cell.
    lane_width : float
        Spacing between scan lines.
    step_size : float
        Spacing between waypoints.
    terrain_alt : float
        Ground elevation (MSL).
    mission_agl : float
        Flight altitude above ground.
    RESULTS_DIR : Path
        Directory for debug plots.
    mission_results_dir : Path
        Directory for QGroundControl missions.
    
    Outputs
    -------
    uav_data_ls : List[Dict]
        UAV path data in ENU.
    uav_data_gps_ls : List[Dict]
        UAV path data in GPS.
    
    Notes
    -----
    - Reduces UAV count to a supported number (1, 2, or 4).
    - Calls other planners recursively after exclusion.
    - Returns None if sub-planners fail.
    '''
    
    # ============================================================================
    #  Compute Cost Matrix for many UAVs
    # ============================================================================
    num_jobs = len(cells)
    cost_matrix = np.zeros((num_uavs, num_jobs))
    
    # Compute costs (similar to multiple_uav_planner)
    all_uav_cell_data = {}
    for m, init_pos_uav in enumerate(init_pos_uav_enu_ls):
        init_pos_uav = init_pos_uav[:2]
        for i, cell in enumerate(cells):
            _, first_point, _, init_dist, second_point, end_points = Get_Start_End_Points([corner_points_ls[i]], init_pos_uav, signal_first_cell=True)
            path_points, path_dist = scan_with_inset(cell, lane_width, step_size, first_point, second_point, end_points, signal_special_scan=True)
            cost_value = path_dist + (init_dist * 10)  # Weighting factor
            cost_matrix[m, i] = cost_value
            
            # Store data (optional, for consistency)
            all_uav_cell_data[(m, i)] = {
                'path_points': path_points,
                'start_end_points': [(path_points[0], path_points[-1])]
            }
    
    # ============================================================================
    #  Find UAV with Highest Total Cost
    # ============================================================================
    cost_per_uav = np.sum(cost_matrix, axis=1)  # Total cost per UAV
    # print(f"Cost per UAV{[m for m in range(num_uavs)]} {cost_per_uav}")
    uav_to_exclude = np.argmax(cost_per_uav)  # Index of UAV with highest cost
    # print(f"UAV {uav_to_exclude} has the highest total cost ({cost_per_uav[uav_to_exclude]:.2f}) and is excluded.")
    
    # ============================================================================
    #  Filter Lists to Exclude the UAV (avoid index shifts)
    # ============================================================================
    filtered_init_pos_uav_enu_ls = [pos for idx, pos in enumerate(init_pos_uav_enu_ls) if idx != uav_to_exclude]
    filtered_init_pos_uavs_gps = [pos for idx, pos in enumerate(init_pos_uavs_gps) if idx != uav_to_exclude]
    num_uavs -= 1

    if num_uavs == 2:
        print("Proceeding with 2 UAVs after exclusion.")
        print(f"Num UAVs now: {num_uavs}")
        # print(f"Init Pos len now: {len(filtered_init_pos_uav_enu_ls)}")
        # ============================================================================
        #  Proceed with 2 UAVs
        # ============================================================================
        results = two_uav_planner(
            num_uavs,
            filtered_init_pos_uav_enu_ls,  # Now has 2 elements
            filtered_init_pos_uavs_gps,    # Now has 2 elements
            reference,
            polygon,
            cells,
            corner_points_ls,
            lane_width,
            step_size,
            terrain_alt,
            mission_agl,
            RESULTS_DIR,
            mission_results_dir
        )
        if results[0] is None:
            print("ERROR: two_uav_planner failed")
            return None, None
        return results
    if num_uavs == 1:
        print("Only 1 UAV left after exclusion, proceeding with single UAV planner.")
        results = one_uav_planner(
            num_uavs,
            filtered_init_pos_uav_enu_ls,  
            filtered_init_pos_uavs_gps,    
            reference,
            polygon,
            cells,
            corner_points_ls,
            lane_width,
            step_size,
            terrain_alt,
            mission_agl,
            RESULTS_DIR,
            mission_results_dir
        )
        if results[0] is None:
            print("ERROR: one_uav_planner failed")
            return None, None
        return results
    if num_uavs == 4:
        print("Proceeding with 4 UAVs after exclusion.")
        results = multiple_uav_planner(
            num_uavs,
            filtered_init_pos_uav_enu_ls,  
            filtered_init_pos_uavs_gps,    
            reference,
            polygon,
            cells,
            corner_points_ls,
            lane_width,
            step_size,
            terrain_alt,
            mission_agl,
            RESULTS_DIR,
            mission_results_dir
        )
        if results[0] is None:
            print("ERROR: multiple_uav_planner failed")
            return None, None
        return results
   

def two_uav_planner(num_uavs, init_pos_uav_enu_ls, init_pos_uavs_gps, reference, polygon, cells, corner_points_ls, lane_width, step_size, terrain_alt, mission_agl, RESULTS_DIR, mission_results_dir):
    '''
    Purpose
    -------
    Assign pairs of cells to 2 UAVs using combinatorial optimization.
    
    Strategy
    -------
    - Generate all possible cell pairs.
    - Compute costs for each UAV-pair assignment.
    - Select the lowest-cost combination.
    - Assign altitudes based on cost.
    - Generate paths and export missions.
    
    Inputs
    ------
    num_uavs : int
        Number of UAVs (must be 2).
    init_pos_uav_enu_ls : List[Tuple[float, float, float]]
        UAV initial positions in ENU.
    init_pos_uavs_gps : List[Tuple[float, float]]
        UAV initial positions in GPS.
    reference : Tuple[float, float, float]
        Reference GPS point.
    polygon : shapely.geometry.Polygon
        Original search area.
    cells : List[Polygon]
        Decomposed cells.
    corner_points_ls : List[List[Tuple[float, float]]]
        Corner points of each cell.
    lane_width : float
        Spacing between scan lines.
    step_size : float
        Spacing between waypoints.
    terrain_alt : float
        Ground elevation (MSL).
    mission_agl : float
        Flight altitude above ground.
    RESULTS_DIR : Path
        Directory for debug plots.
    mission_results_dir : Path
        Directory for QGroundControl missions.
    
    Outputs
    -------
    uav_data_ls : List[Dict]
        UAV path data in ENU.
    uav_data_gps_ls : List[Dict]
        UAV path data in GPS.
    
    Notes
    -----
    - Assumes even cell count; fails if odd.
    - Saves plots and missions.
    '''
    # ============================================================================
    #  Create Cell Sets to assign them to uavs
    # ============================================================================
    cell_set_desired = len(cells) // num_uavs
    if cell_set_desired % 2 or num_uavs > len(cells):
        # odd number of cells or more uavs than cells
        print("One UAV will get no assigned cells")
        return None, None
    cell_set = []
    cell_set_idx = []
    cell_set_corners = []
    for cell_idx_i, cell_i in enumerate(cells):
        for cell_idx_j, cell_j in enumerate(cells):
            if cell_idx_i == cell_idx_j or (cell_idx_i, cell_idx_j) in cell_set_idx or (cell_idx_j, cell_idx_i) in cell_set_idx:
                # no duplicates of sets
                continue
            cell_set.append((cell_i,cell_j))
            cell_set_idx.append((cell_idx_i, cell_idx_j))
            cell_set_corners.append((corner_points_ls[cell_idx_i], corner_points_ls[cell_idx_j]))
    # print(f"Total Cell Sets Created: {len(cell_set)}")
    # print(f"Cell Sets Indices: {cell_set_idx}")
    # print(f"Cell Sets: {cell_set[0]}")

    # ============================================================================
    #  Path Planning for each Set of Cells
    # ============================================================================
    uav_data_ls = []
    first_points_per_set_ls = []
    uav_sets_data_ls = []
    cost_matrix = np.zeros((num_uavs, len(cell_set)))
    for uav_idx, init_pos_uav in enumerate(init_pos_uav_enu_ls):
        init_pos_uav = init_pos_uav[:2]
        # print(f"\n{'='*40}\nUAV {uav_idx} Initial Position: {init_pos_uav}\n{'='*40}")
        uav_data_ls.append({
            'uav_idx': uav_idx,
            'init_pos': init_pos_uav_enu_ls[uav_idx],})
        
        
        set_data_ls = []
        # --- For each Set of Cells ---
        for curr_set_idx, curr_set in enumerate(cell_set):
            path = []
            num_added_wp_cell = []
            # --- Find the first Cell in Set ---
            first_cell_idx, first_point, _, init_dist, second_point, end_points= Get_Start_End_Points(cell_set_corners[curr_set_idx], init_pos_uav, signal_first_cell=True)
            first_points_per_set_ls.append((first_cell_idx,first_point))
    
            # --- Generate Path of first Cell in Set ---
            path_points, path_dist = scan_with_inset(curr_set[first_cell_idx], lane_width, step_size, first_point, second_point, end_points, signal_special_scan=True)
            path.extend(path_points)
            path_set_dist = path_dist
            num_added_wp_cell.append(len(path_points))
            start_end_points_ls = [(path_points[0], path_points[-1])]
            start_sec_end_ls = [(first_point, second_point, path_points[-1])]
            prev_cell_idx_ls = [first_cell_idx]
            
            # --- Generate Path for remaining Cells in the current Set ---
            for i, cell in enumerate(curr_set): 
                if i == first_cell_idx:
                    continue  # Skip first cell (already processed)
                # --- Get Start and End Points for the Next Cell ----
                # print(f"Getting start and end points for cell {i} in set {curr_set_idx}")
                next_cell_idx, next_first_point, _, _, next_second_point, next_end_points_ls = Get_Start_End_Points(
                    cell_set_corners[curr_set_idx], init_pos_uav, signal_first_cell=False, 
                    prev_cell_idx_ls=prev_cell_idx_ls, 
                    end_point_prev=path_points[-1])

                # --- Generate Path for the Next Cell ---
                path_points, path_dist = scan_with_inset(curr_set[next_cell_idx], lane_width, step_size, next_first_point, next_second_point, next_end_points_ls, signal_special_scan=True) 

                path.extend(path_points)
                path_set_dist += path_dist
                num_added_wp_cell.append(len(path_points))
                prev_cell_idx_ls.append(next_cell_idx) # Update for next iteriation
                start_end_points_ls.append((path_points[0], path_points[-1]))  # Track all start and end points
                start_sec_end_ls.append((first_point, second_point, path_points[-1]))
            
            # --- Cost value Calculation for each Set ---
            cost_value = path_set_dist + (init_dist * 10)  # Weighting factor of 10 for init_dist
            cost_matrix[uav_idx,curr_set_idx] = cost_value
            # print(f"UAV {uav_idx}, Set {curr_set_idx} Cells in Set {cell_set_idx[curr_set_idx]} → Path Distance: {path_set_dist:.2f}, Init Distance: {init_dist:.2f}, Cost: {cost_value:.2f}")
            # print(f"Num of Waypoints in Set {num_added_wp_cell}")
            # print(f"\n{'='*20}")


            # --- Store data for each Set ---
            set_data_ls.append({
                'set_idx': curr_set_idx,
                'cells_set_idx': cell_set_idx[curr_set_idx],
                'corner_points_set': cell_set_corners[curr_set_idx],
                'first_cell_set_idx': first_cell_idx,
                'first_cell_set': curr_set[first_cell_idx],
                'init_dist': init_dist,
                'start_sec_end_ls_set': start_sec_end_ls,
                'start_end_points': start_end_points_ls,
                'path_points_set': path,
                'cost_set': cost_value,
                'num_wp': num_added_wp_cell
            })
            
        # --- Store data for each UAV ---
        uav_sets_data_ls.append(set_data_ls)
        # print(f"Number of sets calculated for UAV idx {uav_idx}: {len(set_data_ls)}\n")
        
    # ============================================================================
    #  Combination of Sets
    # ============================================================================
    if num_uavs == 2: 
        # 6 Combinations if 4 Cells and 2 UAVs
        combi_set_idx = []
        combi_cell_idx = []
        combi_cost_ls = []

        # --- Pairing Sets ---
        for i in range(len(uav_sets_data_ls[0])):
            j = len(uav_sets_data_ls[0]) -1 -i
            combi_set_idx.append((uav_sets_data_ls[0][i]['set_idx'], uav_sets_data_ls[1][j]['set_idx']))
            combi_cell_idx.append((uav_sets_data_ls[0][i]['cells_set_idx'], uav_sets_data_ls[1][j]['cells_set_idx']))
            combi_cost = uav_sets_data_ls[0][i]['cost_set'] + uav_sets_data_ls[1][j]['cost_set']
            combi_cost_ls.append(combi_cost)
        # print(f"Cost Matrix for Combinations: \n{combi_cost_ls}\n")

        # --- Find the Set Combination with the Lowest Cost ---
        low_combi_idx, low_cost = min(enumerate(combi_cost_ls), key=lambda x: x[1])
        # print(f"{'='*40}\nLowest Combi found\nCombi idx {low_combi_idx} with Cost: {low_cost}\nSet idx {combi_set_idx[low_combi_idx]}\nCell Combi {combi_cell_idx[low_combi_idx]}")
        low_set_idx = combi_set_idx[low_combi_idx][0]
        # print(f"Lowest Set Idx for uav {0}: {low_set_idx}")
        
        for uav_idx in range(num_uavs):
            # Find the existing UAV data entry/dictionary
            for uav_data in uav_data_ls:
                if uav_data['uav_idx'] == uav_idx:
                    # Update existing dictionary
                    uav_data['path_points'] = uav_sets_data_ls[uav_idx][combi_set_idx[low_combi_idx][uav_idx]]['path_points_set']
                    # print(f"Len of PAth points saved: {len(uav_sets_data_ls[uav_idx][combi_set_idx[low_combi_idx][uav_idx]]['path_points_set'])}")
                    uav_data['start_end_points'] = uav_sets_data_ls[uav_idx][combi_set_idx[low_combi_idx][uav_idx]]['start_end_points']
                    # print(f"Num of WP for UAV {uav_idx}: {uav_sets_data_ls[uav_idx][combi_set_idx[low_combi_idx][uav_idx]]['num_wp']}")
                    break
                
        # --- Init Altitudes based on assigned Cost ---
        # Sort assignments by cost ascending
        uav0_cost = uav_sets_data_ls[0][combi_set_idx[low_combi_idx][0]]['cost_set']
        uav1_cost = uav_sets_data_ls[1][combi_set_idx[low_combi_idx][1]]['cost_set']
        # print(f"Cost UAV0: {uav0_cost}")
        # print(f"Cost UAV1: {uav1_cost}")
        if uav1_cost > uav0_cost:
            init_alt =  [(mission_agl+2), (mission_agl-2)]
            # print(f"{init_alt}")
        else:
            init_alt =  [(mission_agl-2), (mission_agl+2)]
            # init_alt = [alt + 8 for alt in np.arange(1, num_uavs+1, 1)] # Generates a number of evenly spaced values + 8m
            # print(f"{init_alt}")
        uav_data_dict = {uav['uav_idx']: uav for uav in uav_data_ls}
        for uav_idx in range(num_uavs):
            if uav_idx in uav_data_dict:
                uav_data = uav_data_dict[uav_idx]
                uav_data['init_alt'] = init_alt[uav_idx]
                # print(f"UAV {uav_idx} → Init Altitude: {init_alt[uav_idx]:.1f}m")
        
        # ============================================================================
        #  Convert Waypoints in GPS Data for each UAV
        # ============================================================================
        uav_data_gps_ls = convert_in_gps_export(uav_data_ls, reference, init_pos_uavs_gps, mission_agl, 
                                                RESULTS_DIR, mission_results_dir)
        
        # ============================================================================
        #  Plot Results
        # ============================================================================
        plot_results(polygon, lane_width, cells=cells, uav_data_ls=uav_data_ls, results_dir=RESULTS_DIR,
                    corner_points_ls=corner_points_ls)

        return uav_data_ls, uav_data_gps_ls

def run_path_planner(RESULTS_DIR, mission_results_dir, poly_gps, holes_gps, init_pos_uavs_gps, terrain_alt, mission_agl, lane_width, step_size, num_uavs):
    '''
    Purpose
    -------
    Complete GPS-to-Mission path planner for multi-UAV coverage of a search area.
    Converts GPS inputs to ENU coordinates, decomposes the area into cells, assigns UAVs to cells,
    generates lawnmower paths, and exports QGroundControl mission files.
    
    Parameters
    ----------
    RESULTS_DIR : Path
        Directory to save debug plots and intermediate results (e.g., cell decompositions, path visualizations).
    mission_results_dir : Path
        Directory to save QGroundControl .plan mission files for each UAV.
    poly_gps : List[Tuple[float, float]]
        GPS coordinates of the search area polygon (latitude, longitude). 
        For rectangles: 2 diagonal points; for irregular polygons: 3+ points.
    holes_gps : List[List[Tuple[float, float]]]
        No-fly zones inside the search area. Each hole is a list of GPS coordinates (latitude, longitude).
        Empty list if no holes.
    init_pos_uavs_gps : List[Tuple[float, float]]
        Initial GPS positions of UAVs (latitude, longitude). Number of UAVs is inferred from this list if num_uavs is None.
    terrain_alt : float
        Ground elevation at the mission site (MSL, meters). Used as reference altitude for ENU conversions.
    mission_agl : float
        Flight altitude above ground level (meters). Applied to all waypoints.
    lane_width : float
        Spacing between adjacent scan lines (meters). Affects cell decomposition and path density.
    step_size : float
        Spacing between waypoints along each scan line (meters). Controls path resolution.
    num_uavs : int
        Number of UAVs to use. If None, auto-detected from len(init_pos_uavs_gps). Must be compatible with cell count.
    
    Returns
    -------
    uav_data_ls : List[Dict]
        UAV path data in ENU coordinates, including paths, start/end points, and altitudes.
    uav_data_gps_ls : List[Dict]
        UAV path data in GPS coordinates, ready for mission export.
    
    Notes
    -----
    - Auto-detects UAV count if num_uavs is None.
    - Handles various UAV/cell ratios (e.g., excludes UAVs if too many, pairs cells if fewer UAVs).
    - Saves plots to RESULTS_DIR and missions to mission_results_dir.
    - Assumes rectangular or polygonal search areas; holes are subtracted from the area.
    - Fails gracefully with error messages if inputs are invalid (e.g., no cells after decomposition).
    '''
    print(f"\n{'='*60}\nStarting Path Planner\n{'='*60}\n")
    print(f"Input Parameters:\n- Polygon GPS: {poly_gps}\n- Holes GPS: {holes_gps}\n- Init Pos UAVs GPS: {init_pos_uavs_gps}\n- Terrain Altitude (MSL): {terrain_alt}m\n- Mission AGL: {mission_agl}m\n- Lane Width: {lane_width}m\n- Step Size: {step_size}m\n- Number of UAVs: {num_uavs}\n")
    print(f"{'-'*60}\n")

    # Auto-detect number of UAVs
    if num_uavs is None:
        num_uavs = len(init_pos_uavs_gps)

    # ============================================================================
    #  Convert GPS Data in ENU
    # ============================================================================
    # Reference Point
    ref_lat, ref_lon = poly_gps[0]
    reference = (ref_lat, ref_lon, terrain_alt)

    # Convert search area
    poly_corners_enu, pol_alt_ls = convert_search_area_polygon(poly_gps, reference)
    
    # Convert holes
    hole_corners = None
    if holes_gps:
        hole_corners, hole_alt_ls = convert_search_area_polygon(holes_gps, reference)

    # Convert initial Pos UAV
    init_pos_uav_enu_ls = []
    for m, init_pos_uav_gps in enumerate(init_pos_uavs_gps):
        init_pos_uav_enu = convert_gps_point_to_enu(init_pos_uav_gps, reference)
        init_pos_uav_enu_ls.append(init_pos_uav_enu)
    
    # ============================================================================
    #  Create Polygon
    # ============================================================================
    if hole_corners:
        polygon = Polygon(poly_corners_enu, holes=[hole_corners])
    else:
        polygon =  Polygon(poly_corners_enu)
    
    # ============================================================================
    #  Decompose Polygon
    # ============================================================================
    try:
        cells, _, corner_points_ls = get_decomposed_area(polygon, lane_width, num_uavs, RESULTS_DIR)
        print(f"Decomposed into {len(cells)} cells.")
    except Exception as e:
        print(f"ERROR: Polygon decomposition failed: {e}")
        return None, None
    
    if not cells:
        print("ERROR: No valid cells after decomposition")
        return None, None
    
    
    # ============================================================================
    #  Path Points for 1 UAV
    # ============================================================================
    if num_uavs == 1:
        results = one_uav_planner(
            num_uavs,
            init_pos_uav_enu_ls,
            init_pos_uavs_gps,
            reference,
            polygon,
            cells, 
            corner_points_ls, 
            lane_width, 
            step_size,
            terrain_alt,
            mission_agl,
            RESULTS_DIR,
            mission_results_dir)
            

    # ============================================================================
    #  Path Points for multiple UAVs
    # ============================================================================
    elif num_uavs == len(cells):
        print("Path Points for multiple UAVs")
        results = multiple_uav_planner(
            num_uavs,
            init_pos_uav_enu_ls,
            init_pos_uavs_gps,
            reference,
            polygon,
            cells, 
            corner_points_ls, 
            lane_width, 
            step_size,
            terrain_alt,
            mission_agl,
            RESULTS_DIR,
            mission_results_dir)
       
    # # ============================================================================
    # #  Path Points for initial 3 UAVs
    # # ============================================================================
    elif num_uavs == 3 or num_uavs > len(cells):
        print("Path Points for initial 3 UAVs")
        results = too_many_uav_planner(
            num_uavs,
            init_pos_uav_enu_ls,
            init_pos_uavs_gps,
            reference,
            polygon,
            cells, 
            corner_points_ls, 
            lane_width, 
            step_size,
            terrain_alt,
            mission_agl,
            RESULTS_DIR,
            mission_results_dir)
        return results
        
    # ============================================================================
    #  Path Points for more Cells than UAVs
    # ============================================================================
    elif num_uavs < len(cells):
        print("Path Points for more Cells than UAVs")
        results = two_uav_planner(
            num_uavs,
            init_pos_uav_enu_ls,
            init_pos_uavs_gps,
            reference,
            polygon,
            cells, 
            corner_points_ls, 
            lane_width, 
            step_size,
            terrain_alt,
            mission_agl,
            RESULTS_DIR,
            mission_results_dir
            )
        if results[0] is None:
            print("ERROR: two_uav_planner failed due to incompatible UAV/cell count")
            return None, None

        return results
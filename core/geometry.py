"""
Geometry processing: decomposition, scanning, and point selection.
"""

import numpy as np
import math
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.ops import split, unary_union
from pathlib import Path
import matplotlib.pyplot as plt
from PathPlanner.io.visualization import plot_decomposed_cells
from PathPlanner.io.visualization import plot_flyable_area

def get_decomposed_area(polygon, lane_width, num_uavs, RESULTS_DIR):
    '''
    Purpose
    -------
    Decompose a polygon into multiple flyable cells for UAV scanning.
    
    - For polygons without holes: Split into exactly `num_uavs` cells using equal-interval scan lines 
      (horizontal or vertical based on polygon orientation) to ensure balanced coverage.
    - For polygons with holes: Split into 2-4 rectangular regions by cutting vertically (left/right) 
      and horizontally (top/bottom) around the hole, minimizing fragmentation.
    
    Strategy
    -------
    - No Holes: Calculate scan direction based on polygon aspect ratio, generate `num_uavs - 1` 
      splitting lines at equal intervals, and use Shapely's split to divide the polygon.
    - Holes: Perform vertical splits at hole edges, then horizontal splits if needed.
    - All cells are inset by `lane_width / 2` to create flyable areas.
    
    Inputs
    ------
    polygon : shapely.geometry.Polygon
        The search area to decompose (may include holes).
    lane_width : float
        Lane spacing used for inset calculation (meters).
    num_uavs : int
        Number of UAVs/cells to create (affects splitting for no-hole polygons).
    RESULTS_DIR : Path
        Directory to save debug plots.
    
    Outputs
    -------
    flyable_area_ls : List[Polygon]
        List of inset polygons ready for scanning (buffered inward by lane_width/2).
    centroids : List[Point]
        Centroids of the flyable areas.
    corner_points_ls : List[List[Tuple[float, float]]]
        Corner points of each flyable area (excluding the closing point).
    
    Notes
    -----
    - For no holes: Ensures `num_uavs` cells; falls back to original polygon if splitting fails.
    - For holes: May produce 2-4 cells; early returns if splits don't yield expected parts.
    - Debug plots are saved for visualization.
    - Empty cells after inset are skipped.
    '''
    # ============================================================================
    # No Holes Case
    # ============================================================================
    if not polygon.interiors:
        print("No holes detected in the polygon.")
        if num_uavs == 1:
            cells_not_shrunk = [polygon] # continue with shrinking the original polygon because split is not needed
        else:
            print("Proceeding with decomposition for multiple UAVs.")
            # ============================================================================
            # SECTION: Initialize Variables
            # ============================================================================
            min_x, min_y, max_x, max_y = polygon.bounds
            poly_corner_points = list(polygon.exterior.coords[:-1])
            first_point = poly_corner_points[0] # first corner point as reference
            first_idx = 0

            # ============================================================================
            # SECTION: Find Second Point
            # ============================================================================
            # Second Point is the adjacent corner with the longer distance
            for i, corner in enumerate(poly_corner_points):
                if corner == first_point:
                    first_idx = 0
                    break
            if first_idx is None:
                raise ValueError("first_point not found in poly_corner_points")
            
            # Determine Idx of Neighbors Corner Points (Circular list picker)
            n = len(poly_corner_points) 
            neighbor1 = (first_idx - 1) % n  # Previous neighbor
            neighbor2 = (first_idx + 1) % n  # Next neighbor

            # Calculate distances 
            dist_1 = np.linalg.norm(np.array(poly_corner_points[neighbor1])-first_point)
            dist_2 = np.linalg.norm(np.array(poly_corner_points[neighbor2])-first_point)
            # print(f"Distance to Neighbor1 (Idx {neighbor1}): {dist_1}")
            # print(f"Distance to Neighbor2 (Idx {neighbor2}): {dist_2}")

            # Choose the adjacent corner with the longer distance as second_point
            if dist_1 > dist_2:
                second_point = poly_corner_points[neighbor1]
            else:
                second_point = poly_corner_points[neighbor2]

            # ============================================================================
            # SECTION: Find End Points (remaining corners)
            # ============================================================================
            end_points_ls = []
            # Add remaining corners (excluding first_point and second_point)
            for i, corner in enumerate(poly_corner_points):
                if corner == first_point or corner == second_point:
                    continue
                end_points_ls.append(corner)

            # ============================================================================
            # SECTION: Determine Split Parameter and Direction
            # ============================================================================
            # Calculate direction vector between first_point and second_point
            dx = second_point[0] - first_point[0]
            dy = second_point[1] - first_point[1]
            # print(f"\n{'='*80}")
            # print(f"dx = {dx}, dy = {dy}")

            # --- Determine primary scan direction (axis with larger change) & Determine Cell Area and Cell length ---
            if abs(dx) >= abs(dy):
                # we scan via changing y values 
                scan_value_idx = 1 # idx for y
                poly_len = np.linalg.norm(max_y-min_y) # Polygon length along scan direction
                cell_len = poly_len / num_uavs # Initial estimate for Cell length
                step = abs(cell_len) if end_points_ls[0][1] > first_point[1] else -abs(cell_len)   
            else:
                scan_value_idx = 0 # idx for x
                poly_len = np.linalg.norm(max_x-min_x) # Polygon length along scan direction
                cell_len = poly_len / num_uavs # Initial estimate for Cell length
                step = abs(cell_len) if end_points_ls[0][0] > first_point[0] else -abs(cell_len)


            # ============================================================================
            # SECTION: Generate Splitting Lines
            # ============================================================================
            # Generate exactly num_uavs - 1 splitting lines at equal intervals
            num_splits = num_uavs - 1
            scan_lines = []
            min_x, min_y, max_x, max_y = polygon.bounds

            if scan_value_idx == 1:  # Scanning/splitting in y-direction (horizontal lines)
                split_positions = [min_y + i * (max_y - min_y) / num_uavs for i in range(1, num_splits + 1)]
                for pos in split_positions:
                    line = LineString([(min_x - 10, pos), (max_x + 10, pos)])  # Extend beyond bounds to ensure crossing
                    scan_lines.append(line)
            else:  # Scanning/splitting in x-direction (vertical lines)
                split_positions = [min_x + i * (max_x - min_x) / num_uavs for i in range(1, num_splits + 1)]
                for pos in split_positions:
                    line = LineString([(pos, min_y - 10), (pos, max_y + 10)])  # Extend beyond bounds to ensure crossing
                    scan_lines.append(line)

            # Combine and split using the full lines
            if scan_lines:
                multi_line = MultiLineString(scan_lines)
                split_result = split(polygon, multi_line)
                cells_not_shrunk = [geom for geom in split_result.geoms if isinstance(geom, Polygon) and not geom.is_empty]
                # print(f"Decomposed into {len(cells_not_shrunk)} cells using scan-line based splitting.")
                
                # --- Debug Plot ---
                plot_decomposed_cells(cells_not_shrunk, RESULTS_DIR, "Decomposed Cells (No Holes, Scan-Line Based)")
                
    # ============================================================================
    # SECTION: Holes Case
    # ============================================================================
    else:
        print("Hole detected in the polygon, proceeding with decomposition.")
        # --- Initialize Variables (Safety Step) ---
        left_cell = None          # Area LEFT of hole
        right_cell = None         # Area RIGHT of hole
        bottom_cell = None        # Area BELOW hole
        upper_cell = None         # Area ABOVE hole
        
        # Intermediate variables
        rest_cell = None
        middle_cell = None
        rest_middle = None
        
        min_x_def, min_y_def, max_x_def, max_y_def = polygon.bounds # overall polygon bounds
        
        # Get hole bounds (assuming 1st hole)
        hole = polygon.interiors[0]
        h_min_x, h_min_y, h_max_x, h_max_y = hole.bounds # hole bounds


        # ============================================================================
        # SECTION: VERTICAL SPLIT (Left / Rest / Right)
        # ============================================================================
        # Creates vertical cut lines at hole edges to separate left and right areas
        v_line_min_x = LineString([(h_min_x,min_y_def-10), (h_min_x, max_y_def+10)]) # vertical line at beginning of the hole (+-10 to ensure full cut through polygon)
        v_line_max_x = LineString([(h_max_x,min_y_def-10), (h_max_x, max_y_def+10)]) # vertical line at the end of the hole

        # Split at left edge of hole
        split_collec_left = split(polygon, v_line_min_x) # area 1 is between left donut ede and start of hole
        
        # Extract the part to the LEFT of the hole (ready to use)
        for geom in split_collec_left.geoms:
            if geom.centroid.x < h_min_x:
                left_cell = geom #  This is the cell on the left
                
            else:
                rest_cell = geom  # Everything else (will be split again)

        # --- Check for errors in first split ---
        if rest_cell is None:
            print("WARNING: Error in first split, rest_cell is None")
            cells_not_shrunk = [polygon] # continue with shrinking the original polygon
        
        # Split at right edge of hole
        split_collec_right = split(rest_cell, v_line_max_x)

        for geom in split_collec_right.geoms: 
            if geom.centroid.x > h_max_x:
                right_cell = geom # This is the cell on the right
            else: 
                middle_cell = geom # This is the middle part surrounding the hole
        
        # --- Early Return Check after VERTICAL SPLIT ---
        # Only return early if we got MORE than 2 pieces (i.e., hole already split vertically into 3+ parts)
        if len(split_collec_right.geoms) > 2:
            print("We have already 3+ parts...")
            # Hole already split vertically into 3+ pieces; skip horizontal split
            cells_not_shrunk = [left_cell] + [p for p in split_collec_right.geoms]
        
        # ============================================================================
        # SECTION:  HORIZONTAL SPLIT (Bottom / Top)
        # ============================================================================    
        # If we still have 2 or less pieces after vertical split, we need to split horizontally too
        elif len(split_collec_right.geoms) <= 2:
            print("We have to split horizontally too")
            
            # Create horizontal cut lines at hole edges to separate bottom and top areas
            h_line_min_y = LineString([(h_min_x-10, h_min_y), (h_max_x+10, h_min_y)])
            h_line_max_y = LineString([(h_min_x-10, h_max_y), (h_max_x+10, h_max_y)])

            # Check for errors in middle_cell
            if middle_cell is None:
                print("WARNING: middle_cell is None")
                cells_not_shrunk = [left_cell] + [p for p in split_collec_right.geoms] # continue with shrinking what we have

            # Split at bottom edge of hole    
            split_collec_bottom = split(middle_cell, h_line_min_y)
            for geom in split_collec_bottom.geoms:
                if geom.centroid.y < h_min_y:
                    bottom_cell = geom # This is the bottom cell
                else:
                    rest_middle = geom # Everything else (to be split again)

            # Check for errors in rest_middle
            if rest_middle is None:
                print("WARNING: rest_middle is None")
                cells_not_shrunk = [p for p in (left_cell, bottom_cell, right_cell) if p is not None] # continue with shrinking what we have
                
            # Split at top edge of hole
            split_collec_top = split(rest_middle, h_line_max_y)
            for geom in split_collec_top.geoms:
                if geom.centroid.y > h_max_y:
                    upper_cell = geom # This is the upper cell
            if upper_cell is None:
                print("WARNING: upper_cell is None after top split")
            
            # if we reach here, we cut 4 times and have all 4 parts
            cells_not_shrunk = [p for p in (left_cell, bottom_cell, upper_cell, right_cell) if p is not None] # stores all the parts before shrinking
            
    # Safety Check: ensure we have at least one cell
    if not cells_not_shrunk:
        print("WARNING: No valid cells after decomposition")
        cells_not_shrunk = [polygon]  # Fallback to original

    # ============================================================================
    # SHRINKING SECTION: Runs for ALL code paths
    # ============================================================================
    flyable_area_ls = []
    corner_points_ls = []
    for i in range(len(cells_not_shrunk)):
        inset_dist = lane_width / 2
        flyable_area = cells_not_shrunk[i].buffer(-inset_dist)
        
        if flyable_area.is_empty:
            print(f"WARNING: Cell {i} became empty after buffering, skipping...")
            continue
        
        flyable_area_ls.append(flyable_area)
        
        # --- Debug Plot ---
        plot_flyable_area(flyable_area, RESULTS_DIR, i)
        
        corner_points = list(flyable_area.exterior.coords[:-1])
        corner_points_ls.append(corner_points)

    
    return flyable_area_ls, [p.centroid for p in flyable_area_ls], corner_points_ls 

def scan_with_inset(cell, lane_width, step_size, first_point=None, second_point=None, end_points=None, signal_special_scan=False):
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
    # ============================================================================
    # SECTION: Input Validation
    # ============================================================================
    if cell.is_empty:
        print("WARNING: Cell is empty, returning no path points.")
        return [], []
    
    # ============================================================================
    # SECTION: First Cell Scan (with specified start and end points)
    # ============================================================================
    if signal_special_scan and first_point is not None and second_point is not None and end_points is not None:
        # print("Scan: Starting at specified start point.")
        if not end_points:
            print("ERROR: end_points is empty or invalid")
            return []
        
        # Initialize sweep state
        direction = 1  # 1 = forward, -1 = backward. Will toggle after every row.
        idx_current_row = 0
        
        # ============================================================================
        # SECTION: Determine Scan Parameter and Direction
        # ============================================================================
        # # Calculate direction vector between first_point and second_point
        dx = second_point[0] - first_point[0]
        dy = second_point[1] - first_point[1]
        # print(f"\n{'='*80}")
        # print(f"dx = {dx}, dy = {dy}")
        # # Determine primary scan direction (axis with larger change)
        if abs(dx) >= abs(dy):
            # we scan via changing y values
            scan_value_idx = 1 # idx for y
            print(f"Scan idx: {scan_value_idx}") 
            step = abs(lane_width) if end_points[0][1] > first_point[1] else -abs(lane_width)
            
        else:
            scan_value_idx = 0 # idx for x
            print(f"Scan idx: {scan_value_idx}") 
            step = abs(lane_width) if end_points[0][0] > first_point[0] else -abs(lane_width)

        # --- Perform the scan ---
        path_points = []  # flat list of (x, y) waypoints for this cell
        min_x, min_y, max_x, max_y = cell.bounds
        
        # --- Determine whether to start from First Point or Second Point ---
        # print(f"\n{'='*80}")
        # print(f"DEBUG: Checking start_point logic")
        # print(f"  direction = {direction}")
        # print(f"  scan_value_idx = {scan_value_idx}")
        # print(f"  first_point = {first_point}")
        # print(f"  second_point = {second_point}")
        # print(f"{'='*80}\n")

        # We have to check wheter the second step i on the line with first_line 
        # True -> normal search lines.... no edit needit
        # False -> No rectangular Poly -> If Second[0] < first[0] while step > 0 than we have also to create serach path into other direction 
        if scan_value_idx == 0:
            if step > 0:
                target_axis = max_x
            else:
                target_axis = min_x

            if first_point[0] == second_point[0] or first_point[0] < second_point[0] and step > 0 or first_point[0] > second_point[0] and step < 0:
                print("WARNING: Proceeding with first point")
                # proceed as normal
                start_point = first_point
                # path_points.append(first_point)
                
            elif first_point[0] > second_point[0] and step > 0 or first_point[0] < second_point[0] and step < 0:
                print("WARNING: Start at second point")
                start_point = second_point # start scan at second point 
                path_points.append(first_point) 
                path_points.append(start_point)
                   
        if scan_value_idx == 1:
            if step > 0:
                target_axis = max_y
            else:
                target_axis = min_y
            if first_point[1] == second_point[1] or first_point[1] < second_point[1] and step > 0 or first_point[1] > second_point[1] and step < 0:
                # proceed as normal
                start_point = first_point
                # path_points.append(start_point)
                print("WARNING: Proceeding with first point")

            elif first_point[1] > second_point[1] and step > 0 or first_point[1] < second_point[1] and step < 0:
                print("WARNING: Start at second point")
                start_point = second_point
                # start scan at second point 
                # if the second point is not included in a normal start scan at first point then we have to start at second point
                path_points.append(first_point) 
                path_points.append(start_point)
                

        if scan_value_idx == 0:  # Scanning Vertical (Changing Y)
            # Compare start_point Y to the geometric center or the other point
            # If start Y is greater than the other end, we are starting at Top -> Must go Down (-1)
            if start_point[1] > second_point[1] if start_point is first_point else start_point[1] > first_point[1]:
                direction = -1
            else:
                direction = 1

        else:  # Scanning Horizontal (Changing X)
            # If start X is greater than the other end, we are starting at Right -> Must go Left (-1)
            if start_point[0] > second_point[0] if start_point is first_point else start_point[0] > first_point[0]:
                direction = -1
            else:
                direction = 1

        if start_point == second_point:
            direction *= -1  # Reverse direction if starting at second point
        
        print(f"DEBUG: Initial Direction set to {direction}")
                
        # Define values for scan lines based on first_point and end_points
        scan_values = []
        start_axis = start_point[scan_value_idx]
        edge_threshold = lane_width * 0.8  # To avoid overshooting the edge
        scan_values.append(start_axis + 0.001*np.sign(step)) # np.sign to handle offset direction
        next_scan_value = start_axis + 0.001*np.sign(step) + step  # Next: 97.499 + (-5) = 92.499
        limit = target_axis - edge_threshold * np.sign(step) # Limit to just before the target edge
        while (step > 0 and next_scan_value < limit) or (step < 0 and next_scan_value > limit):
            scan_values.append(next_scan_value)
            next_scan_value += step
        scan_values.append(limit)  # ensure last lane is on the opposite edge
        # Append final scan value at the edge
        scan_values.append(target_axis)
        print(f"Scan Values: {scan_values}")

        # --- Perform the remaining scan lines ---
        num_seg = 0
        min_x, min_y, max_x, max_y = cell.bounds
        for curr_scan_value in scan_values:
            
            if scan_value_idx == 0: # scan via x values 
                line = LineString([(curr_scan_value, min_y-10), (curr_scan_value, max_y+10)]) 
                intersection = line.intersection(cell)
            else: # scan via y values
                line = LineString([(min_x-10, curr_scan_value), (max_x+10, curr_scan_value)]) 
                intersection = line.intersection(cell)

            if not intersection.is_empty: # proceed only if there is an intersection, meaning the line crosses the polygon border

                # Robust Segment Extraction
                if intersection.geom_type == 'LineString':
                    seg = intersection
                elif intersection.geom_type == 'MultiLineString':
                    seg = max(intersection.geoms, key=lambda s: s.length)
                else:
                    seg = None

                if seg: # only runs if seg is valid
                    row_points = []
                    
                    
                    # Always include exact start point
                    pt = seg.interpolate(0)
                    row_points.append((pt.x, pt.y))
                    
                    # Sample intermediate points with regular spacing
                    dist = step_size
                    while dist < seg.length - 0.5:  # Stop before end (0.5m margin)
                        pt = seg.interpolate(dist)
                        row_points.append((pt.x, pt.y))
                        dist += step_size
                    
                    # Always include exact end point (at boundary)
                    if seg.length > 0.1:  # Only if segment is non-trivial
                        pt = seg.interpolate(seg.length)
                        row_points.append((pt.x, pt.y))
                    
                        
                    # If direction is backwards (-1), flip this row
                    if direction == -1:
                        row_points.reverse()
                    
                    # Add to main list
                    path_points.extend(row_points)

                    # Counter for segments/turns
                    num_seg += 1
   
            direction *= -1 # Flip direction for the next row

        # --- Path Distance Calculation ---
        path_dist = 0
        for i in range(1, len(path_points)):
            p1 = np.array(path_points[i-1])
            p2 = np.array(path_points[i])
            path_dist += np.linalg.norm(p2 - p1)

        
        return path_points, path_dist

    # ============================================================================
    # SECTION: Standard Cell Scan (without specified start/end points)
    # ============================================================================
    else:
        print("WARNING: Default Cell Scan, something went wrong...")
        
        min_x, min_y, max_x, max_y = cell.bounds
        scan_y_values = []
        scan_y_values.append(min_y + 0.001) # First Row on the bottom with offset
        
        next_y = min_y + lane_width
        while next_y < max_y:
            if max_y - next_y > 0.001:  # next_y is still within bounds
                scan_y_values.append(next_y) # and add to the list for scanning, it holds all the y values for the scan lines
            next_y += lane_width

        scan_y_values.append(max_y - 0.001) # this ensures that the last lane is on the edge
        idx_last_seg = len(scan_y_values) # idx to highlight the last segmnet

        path_points = []  # flat list of (x, y) waypoints for this cell
        direction = -1 # 1 = Left->Right, -1 = Right->Left
        num_seg = 0
       

        for current_y in scan_y_values:
            idx_current_row += 1
            line = LineString([(min_x-10, current_y), (max_x+10, current_y)])
            intersection = line.intersection(cell)

            if not intersection.is_empty: # proceed only if there is an intersection, meaning the line crosses the polygon border

                # Robust Segment Extraction
                if intersection.geom_type == 'LineString':
                    seg = intersection
                elif intersection.geom_type == 'MultiLineString':
                    seg = max(intersection.geoms, key=lambda s: s.length)
                else:
                    seg = None

                if seg: # only runs if seg is valid
                    row_points = []
                    dist = 0
                    while dist <= seg.length: # it moves along the segment
                        pt = seg.interpolate(dist) # gets the point(x,y) at the specific distance
                        row_points.append((pt.x, pt.y))
                        dist += 3

                    # If direction is backwards (-1), flip this row
                    if direction == -1:
                        row_points.reverse()
                    
                    # Add to main list
                    path_points.extend(row_points)

                    # Counter for segments/turns
                    num_seg += 1
                    
            direction *= -1 # Flip direction for the next row
        # --- Path Distance Calculation ---
        path_dist = 0
        for i in range(1, len(path_points)):
            p1 = np.array(path_points[i-1])
            p2 = np.array(path_points[i])
            path_dist += np.linalg.norm(p2 - p1)

        
        return path_points, path_dist

def Get_Start_End_Points(corner_points_ls, init_pos_uav, signal_first_cell=False, prev_cell_idx_ls=None, end_point_prev=None):
    '''
    Purpose
    -------
    Determine start and end points for UAV scanning in cells.
    
    Strategy
    -------
    - For first cell: Find nearest corner to UAV position, select adjacent corner as second point.
    - For subsequent cells: Find nearest corner to previous cell's end point, select adjacent corner.
    - Identify remaining corners as end points.
    
    Inputs
    ------
    corner_points_ls : List[List[Tuple[float, float]]]
        Corner points of all cells.
    init_pos_uav : Tuple[float, float]
        UAV initial position.
    signal_first_cell : bool
        True for first cell selection.
    prev_cell_idx_ls : List[int] or None
        Indices of previously visited cells.
    end_point_prev : Tuple[float, float] or None
        End point of previous cell.
    
    Outputs
    -------
    next_cell_idx : int
        Index of selected cell.
    first_point : Tuple[float, float]
        Start point in selected cell.
    first_point_idx : int
        Index of first_point.
    first_dist : float
        Distance to first_point.
    second_point : Tuple[float, float]
        Adjacent corner point.
    end_points_ls : List[Tuple[float, float]]
        Remaining corner points.
    
    Notes
    -----
    - Uses Euclidean distance for nearest point.
    - Assumes rectangular cells with 4 corners.
    '''
    
    # --- The first Cell ---
    if signal_first_cell:
        # ============================================================================
        # SECTION: Find First Point
        # ============================================================================
        # Find the nearest Corner Point to the initial position of the UAV
        best = (float("inf"), None, None)  # (dist, cell_idx, corner_idx), the distance is set to infinity initially
        init_pos = np.array(init_pos_uav) # convert to numpy array for easier calculations
    
        for cell_idx, corners_in_cell in enumerate(corner_points_ls): # loops through each cell, each loop giving a list of four corner points the current cell
            for corner_idx, corner in enumerate(corners_in_cell): # loops through each corner point of the current cell
                d = np.linalg.norm(np.array(corner) - init_pos) # for each corner calculate euclidean distance to the initial position
                if d < best[0]: # if the calculated distance is less than the current best distance found so far, best is updated
                    best = (d, cell_idx, corner_idx)

        # Store the first point information
        first_dist, first_cell_idx, first_point_idx = best
        first_point = corner_points_ls[first_cell_idx][first_point_idx] # tuple(x,y)

        # Get the corners of the first cell
        first_cell_corners = corner_points_ls[first_cell_idx] # list of four tuples
        
        # ============================================================================
        # SECTION: Find Second Point
        # ============================================================================
        # Second Point is the adjacent corner with the longer distance
        for i, corner in enumerate(first_cell_corners):
            if corner == first_point:
                first_idx = i
                break
        if first_idx is None:
            raise ValueError("first_point not found in first_cell_corners")

        # Determine Idx of Neighbour Corner Points (Circular list picker)
        n = len(first_cell_corners) 
        neighbor1 = (first_idx - 1) % n  # Previous neighbor
        neighbor2 = (first_idx + 1) % n  # Next neighbor
        # Calculate distances 
        dist_1 = np.linalg.norm(np.array(first_cell_corners[neighbor1])-first_point)
        dist_2 = np.linalg.norm(np.array(first_cell_corners[neighbor2])-first_point)

        # Choose the adjacent corner with the longer distance as second_point
        if dist_1 > dist_2:
            second_point = first_cell_corners[neighbor1]
        else:
            second_point = first_cell_corners[neighbor2]

        # ============================================================================
        # SECTION: Find End Points (remaining corners)
        # ============================================================================
   
        # Store the possible End Points
        end_points_ls = []    
        # Add the longest point itself as the first endpoint
        for i, corner in enumerate(first_cell_corners):
            if corner == first_point or corner == second_point:
                continue
            end_points_ls.append(corner)
        
        print(f"First Cell Index: {first_cell_idx}, First Point: {first_point}, Second Point: {second_point}, End Points: {end_points_ls}")
        # Return the first point and the two points on the opposite edge as possible end points
        return first_cell_idx, first_point, first_point_idx, first_dist, second_point, end_points_ls
    
    # ============================================================================
    # SECTION: The remaining Cells
    # ============================================================================
    else:
        if end_point_prev is None or prev_cell_idx_ls is None:
            print("ERROR: end_point_prev or prev_cell_idx_ls is None")
            return None, None, None, None, None, None

        # ============================================================================
        # SECTION: Find First Point
        # ============================================================================
        # Find the nearest Corner Point to the Endpoint of the previous Cell
        best = (float("inf"), None, None)  # (dist, cell_idx, corner_idx), the distance is set to infinity initially
        end_point_prev_arr = np.array(end_point_prev)

        for cell_idx, corners_in_cell in enumerate(corner_points_ls): # loops through each cell, each loop giving a list of four corner points the current cell
            # Skip ALL previously visited cells
            if cell_idx in prev_cell_idx_ls:
                continue
            for corner_idx, corner in enumerate(corners_in_cell): # loops through each corner point of the current cell
                d = np.linalg.norm(np.array(corner) - end_point_prev_arr) # for each corner calculate euclidean distance to the initial position
                if d < best[0]: # if the calculated distance is less than the current best distance found so far, best is updated
                    best = (d, cell_idx, corner_idx)
        # Store the first point information
        first_dist, next_cell_idx, first_point_idx = best
        first_point = corner_points_ls[next_cell_idx][first_point_idx] # tuple(x,y)

        # Get the corners of the first cell
        next_cell_corners = corner_points_ls[next_cell_idx] # list of four tuples

    # ============================================================================
    # SECTION: Find Second Point (adjacent corner with longer distance)
    # ============================================================================
    # Find the index of first_point in the corners list
    first_idx = None
    for i, corner in enumerate(next_cell_corners):
        if corner == first_point:
            first_idx = i
            break

    if first_idx is None:
        raise ValueError("first_point not found in next_cell_corners")

    # Determine adjacent corner indices (circular)
    n = len(next_cell_corners)
    neighbor1 = (first_idx - 1) % n  # Previous neighbor
    neighbor2 = (first_idx + 1) % n  # Next neighbor

    # Calculate distances
    dist_1 = np.linalg.norm(np.array(next_cell_corners[neighbor1]) - np.array(first_point))
    dist_2 = np.linalg.norm(np.array(next_cell_corners[neighbor2]) - np.array(first_point))

    # Choose the adjacent corner with the longer distance as second_point
    if dist_1 > dist_2:
        second_point = next_cell_corners[neighbor1]
        second_idx = neighbor1
    else:
        second_point = next_cell_corners[neighbor2]
        second_idx = neighbor2

    # ============================================================================
    # SECTION: Find End Points (remaining corners)
    # ============================================================================
    end_points_ls = []
    
    
    # Add remaining corners (excluding first_point and second_point)
    for i, corner in enumerate(next_cell_corners):
        if corner == first_point or corner == second_point:
            continue
        end_points_ls.append(corner)
  
    print(f"Next Cell Index: {next_cell_idx}, First Point: {first_point}, Second Point: {second_point}, End Points: {end_points_ls}")
    # Return the required values
    return next_cell_idx, first_point, first_point_idx, first_dist, second_point, end_points_ls
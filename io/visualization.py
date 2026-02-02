"""
Visualization functions.
"""
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

def plot_results(original_poly, lane_width, filename="scan_result.png", cells=None, uav_data_ls=None, results_dir=None, corner_points_ls=None):
    '''
    Purpose
    -------
    Visualize polygon, cells, and UAV paths.
    
    Inputs
    ------
    original_poly : Polygon
        Search area.
    lane_width : float
        Lane width.
    cells : List[Polygon] or None
        Decomposed cells.
    uav_data_ls : List[Dict] or None
        UAV data.
    results_dir : Path or None
        Output directory.
    corner_points_ls : List[List[Tuple]] or None
        Cell corners.
    
    Outputs
    -------
    Saves plot to results_dir.
    
    Notes
    -----
    - Shows boundaries, cells, paths, and start/end points.
    '''
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Original boundary
    x, y = original_poly.exterior.xy
    ax.plot(x, y, color='black', linewidth=3, label='Real Boundary')
    ax.fill(x, y, color='gray', alpha=0.1)
    
    # Draw holes
    for interior in original_poly.interiors:
        ix, iy = interior.xy
        ax.plot(ix, iy, color='black', linewidth=3)
        ax.fill(ix, iy, color='white')

    # Draw cells
    if cells:
        colors_cell = ['yellow', 'cyan', 'lime', 'magenta', 'orange', 'pink']
        for i, cell in enumerate(cells):
            color = colors_cell[i % len(colors_cell)]
            cx, cy = cell.exterior.xy
            ax.fill(cx, cy, color=color, alpha=0.25, edgecolor='blue', linewidth=2)
            ax.plot(cx, cy, color='blue', linewidth=1, linestyle='--', dashes=(5, 5))
            centroid = cell.centroid
            ax.text(centroid.x, centroid.y, f"Cell {i}", 
                fontsize=8, ha='center', weight='bold',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8))
    
    # Draw corner points
    if corner_points_ls:
        for cell_idx, corners in enumerate(corner_points_ls):
            for corner_idx, (x, y) in enumerate(corners):
                # Add label only for the first corner point to show in legend
                if cell_idx == 0 and corner_idx == 0:
                    ax.plot(x, y, marker='D', color='darkblue', markersize=5, 
                           markeredgecolor='white', markeredgewidth=1.5, zorder=15, label='Corner Points')
                else:
                    ax.plot(x, y, marker='D', color='darkblue', markersize=5, 
                           markeredgecolor='white', markeredgewidth=1.5, zorder=15)
               
    
    # Plot UAV paths (single or multiple)
    if uav_data_ls:
        colors_uav = ['red', 'blue', 'green', 'purple', 'orange', 'brown']
        
        for uav_idx, uav_data in enumerate(uav_data_ls):
            color_uav = colors_uav[uav_idx % len(colors_uav)]
            
            # Plot path
            if 'path_points' in uav_data and uav_data['path_points']:
                px = [p[0] for p in uav_data['path_points']]
                py = [p[1] for p in uav_data['path_points']]
                ax.plot(px, py, color=color_uav, linewidth=2, 
                       label=f"UAV {uav_data['uav_idx']} Path", alpha=0.8)
                ax.plot(px, py, color=color_uav, marker='.', markersize=8, linestyle='none')
            
            if 'init_pos' in uav_data and 'path_points' in uav_data and uav_data['path_points']:
                init_x, init_y, _ = uav_data['init_pos']
                first_x, first_y = uav_data['path_points'][0]
                ax.plot([init_x, first_x], [init_y, first_y], color=color_uav, 
                    linestyle='--', linewidth=2, alpha=0.7, 
                    label=f"UAV {uav_data['uav_idx']} Travel to Start" if uav_idx == 0 else "")
                # Optional: Add arrowhead
                ax.arrow(init_x, init_y, first_x - init_x, first_y - init_y, 
                        head_width=5, head_length=5, fc=color_uav, ec=color_uav, alpha=0.7)
            

            # Plot start/end points
            if 'start_end_points' in uav_data and uav_data['start_end_points']:
                for pt_idx, (start_pt, end_pt) in enumerate(uav_data['start_end_points']):
                    # Start point (circle)
                    ax.plot(start_pt[0], start_pt[1], marker='o', color=color_uav, 
                           markersize=14, markeredgecolor='black', markeredgewidth=2, zorder=10)
                    ax.text(start_pt[0]-lane_width/4, start_pt[1]+lane_width/4, 
                           f"UAV{uav_data['uav_idx']}-S", fontsize=10, color=color_uav, weight='bold')
                    
                    # End point (X)
                    ax.plot(end_pt[0], end_pt[1], marker='x', color=color_uav, 
                           markersize=14, markeredgewidth=4, zorder=10)
                    ax.text(end_pt[0]-lane_width/4, end_pt[1]+lane_width/4, 
                           f"UAV{uav_data['uav_idx']}-E", fontsize=10, color=color_uav, weight='bold')
            
            # Plot initial position
            if 'init_pos' in uav_data:
                ax.plot(uav_data['init_pos'][0], uav_data['init_pos'][1], 
                       marker='*', color=color_uav, markersize=12, 
                       markeredgecolor='black', markeredgewidth=2, zorder=11,
                       label=f"UAV {uav_data['uav_idx']} Init Pos")

    ax.set_title(f"Multi-UAV Path Planning (Lane Width = {lane_width}m)")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=8, framealpha=0.9)
    ax.grid(True)
    ax.set_aspect('equal')
    
    filepath = results_dir / filename  
    plt.savefig(str(filepath), bbox_inches='tight', dpi=150)
    print(f"Plot saved to {filepath}")
    plt.close()

def plot_decomposed_cells(cells_not_shrunk, RESULTS_DIR, title="Decomposed Cells"):
    '''
    Plot and save debug visualization of decomposed cells.
    
    Inputs
    ------
    cells_not_shrunk : List[Polygon]
        List of cell polygons.
    RESULTS_DIR : Path
        Directory to save the plot.
    title : str
        Plot title.
    '''
    fig, ax = plt.subplots(figsize=(10, 8))
    colors = ['yellow', 'cyan', 'lime', 'magenta', 'orange', 'pink']
    for i, cell in enumerate(cells_not_shrunk):
        color = colors[i % len(colors)]
        if isinstance(cell, Polygon):
            cx, cy = cell.exterior.xy
            ax.fill(cx, cy, color=color, alpha=0.3, edgecolor='blue', linewidth=2)
            ax.plot(cx, cy, color='blue', linewidth=1)
            centroid = cell.centroid
            ax.text(centroid.x, centroid.y, f"Cell {i}\nArea: {cell.area:.1f}", fontsize=8, ha='center', weight='bold',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8))
    ax.set_title(title)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.grid(True)
    ax.set_aspect('equal')
    debug_path = RESULTS_DIR / "debug_decomposed_cells_no_holes.png"
    plt.savefig(str(debug_path), bbox_inches='tight', dpi=150)
    plt.close()

def plot_flyable_area(flyable_area, RESULTS_DIR, i):
    '''
    Plot and save debug visualization of a flyable area after shrinking.
    
    Inputs
    ------
    flyable_area : Polygon
        The flyable area polygon.
    RESULTS_DIR : Path
        Directory to save the plot.
    i : int
        Cell index.
    '''
    fig, ax = plt.subplots(figsize=(10, 8))
    x, y = flyable_area.exterior.xy
    ax.fill(x, y, edgecolor='blue', facecolor='cyan', alpha=0.3)
    ax.set_title(f"Flyable Area {i} after Shrinking")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.yaxis.set_major_locator(plt.MultipleLocator(50)) 
    ax.xaxis.set_major_locator(plt.MultipleLocator(50))
    ax.grid(True)
    ax.set_aspect('equal')
    
    debug_path = RESULTS_DIR / f"debug_flyable_area_{i}.png"
    plt.savefig(str(debug_path), bbox_inches='tight', dpi=150)
    plt.close()


def plot_assignment_results(task_assign_ls, results_dir):
    '''
    Purpose
    -------
    Visualize the results of UAV-cell task assignment using a bar chart of costs.
    
    Inputs
    ------
    task_assign_ls : List[Tuple[int, int, float]]
        List of tuples (uav_idx, cell_idx, cost) representing the assigned UAVs, cells, and costs.
    results_dir : Path
        Directory to save the plot.
    
    Outputs
    -------
    Saves a bar chart plot to results_dir / "task_assignment.png".
    
    Notes
    -----
    - Each bar represents an assignment with height equal to the cost.
    - Colors differentiate UAVs.
    - Includes cost labels on bars.
    '''
    # Create a simple plot to visualize the assignment
    fig, ax = plt.subplots(figsize=(8, 6))
    colors_uav = ['red', 'blue', 'green', 'purple', 'orange', 'brown']

    for i, (uav_idx, cell_idx, cost) in enumerate(task_assign_ls): # for each uav
        ax.barh(i, cost, color=colors_uav[uav_idx % len(colors_uav)], label=f"UAV {uav_idx} → Cell {cell_idx}")
        ax.text(cost + 10, i, f"Cost: {cost:.2f}", va='center')

    ax.set_xlabel("Total Cost")
    ax.set_ylabel("Assignments")
    ax.set_title("Task Assignment Results")
    ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=8, framealpha=0.9)
    plt.tight_layout()
    plt.savefig(results_dir / "task_assignment.png", dpi=150)
    plt.close()
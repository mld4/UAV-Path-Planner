# PathPlanner

**PathPlanner** is a professional Python package for autonomous UAV (Unmanned Aerial Vehicle) path planning. It generates optimized lawnmower-style flight paths for multi-UAV coverage of search areas, handling complex polygons with holes, altitude assignments, and QGroundControl mission exports. Designed for applications like avalanche search, agriculture, or surveillance, it ensures efficient, collision-free paths with cost-based UAV assignment.

## Features
- **Polygon Decomposition**: Splits search areas into flyable cells, accounting for holes and UAV constraints.
- **Path Generation**: Creates back-and-forth (lawnmower) scanning paths with adjustable lane width and step size.
- **Multi-UAV Support**: Optimizes UAV assignment using the Hungarian algorithm for equal or unequal cell counts.
- **GPS Integration**: Converts ENU coordinates to GPS for real-world missions.
- **Mission Export**: Generates QGroundControl-compatible .plan files.
- **Visualization**: Plots decomposition, paths, and assignments for debugging.
- **Modular Design**: Clean separation of core logic, I/O, utilities, and tests.

## Project Structure
```
PathPlanner/
├── __init__.py              # Package initialization and exports
├── __main__.py              # Entry point for running as a module
├── main.py                  # Main script with default test inputs
├── core/                    # Core logic
│   ├── assignment.py        # UAV-cell assignment (Hungarian algorithm)
│   ├── geometry.py          # Polygon decomposition and scanning
│   └── planners.py          # Path planning algorithms
├── io/                      # Input/Output operations
│   ├── mission_export.py    # QGroundControl mission export
│   └── visualization.py     # Plotting and visualization
├── utils/                   # Utilities
│   └── conversions.py       # GPS/ENU conversions
├── tests/                   # Test configurations
│   └── test_cases.py        # Predefined test cases with GPS data
├── results/                 # Output directory for plots and logs
└── test_results/            # Secondary folder for test results (e.g., mission files, plots from test runs)
```

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/PathPlanner.git
   cd PathPlanner
   ```

2. **Install Dependencies**:
   - Ensure Python 3.8+ and pip are installed.
   - Install required packages:
     ```bash
     pip install numpy shapely scipy matplotlib pymap3d
     ```
   - For development, use `pip install -e .` if a `setup.py` is added.

3. **Optional: QGroundControl Setup**:
   - Set the mission export directory (default is `PathPlanner/missions/`):
     ```bash
     export QGC_MISSIONS_DIR="/path/to/your/QGroundControl/Missions"
     ```

## Usage
### Basic Run
Run with default test inputs (simple rectangle, 1 UAV):
```bash
python -m PathPlanner
```
This generates paths, plots, and mission files in `results/` and `missions/`.

### Custom Test Cases
Use predefined test cases from `tests/test_cases.py`:
```python
from tests.test_cases import test_case_stubaier
from main import run_path_planner

# Load test case
config = test_case_stubaier
run_path_planner(**config)
```

### API Usage
Import and use functions directly:
```python
from PathPlanner.core.planners import run_path_planner

# Define inputs
poly_gps = [(0, 0), (1, 0), (1, 1), (0, 1)]  # Search area
holes_gps = []  # No holes
init_pos_uavs_gps = [(0.5, 0.5)]  # UAV position
# ... other params ...

results = run_path_planner(
    RESULTS_DIR=Path("results"),
    mission_results_dir=Path("missions"),
    poly_gps=poly_gps,
    holes_gps=holes_gps,
    init_pos_uavs_gps=init_pos_uavs_gps,
    terrain_alt=0,
    mission_agl=10,
    lane_width=10,
    step_size=10,
    num_uavs=1
)
```

## Test Results
Test results are stored in the `test_results/` folder, including:
- **Mission Files**: QGroundControl .plan files for each UAV.
- **Plots**: Decomposition visualizations, path plots, and assignment diagrams.
- **Logs**: Console output with path distances and costs.

Example test results for the Stubaier Gletscher case (1 UAV):
- Mission file: `UAV0_mission.plan`
- Plots: `debug_decomposed_cells.png`, `path_plot.png`
- Path distance: ~500m, optimized for coverage.

Run tests and check `test_results/` for outputs.

## Configuration
- **Environment Variables**:
  - `QGC_MISSIONS_DIR`: Path to QGroundControl missions folder.
- **Test Cases**: Edit `tests/test_cases.py` to add custom GPS data.
- **Parameters**: Adjust lane width, step size, and altitudes in function calls.

## Contributing
1. Fork the repo.
2. Create a feature branch.
3. Add tests in `tests/`.
4. Submit a pull request.

## License
MIT License. See `LICENSE` for details.

## Changelog
- **v3.0**: Modular refactor, improved decomposition, multi-UAV optimization, professional structure.

For issues or questions, open a GitHub issue!
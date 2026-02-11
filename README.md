# PathPlanner

## Introduction

This project was developed as a semester project to explore path planning for multi-drone search and rescue (SAR) missions. It serves as a foundational tool for planning efficient search trajectories for 1–4 drones to systematically cover predefined areas. In real-world applications, the input could be simplified to GPS coordinates defining the search region.

The project focuses on autonomous drone path planning for SAR operations, where drones must cover areas efficiently while avoiding obstacles and optimizing criteria such as distance and altitude. The implementation decomposes search polygons, assigns tasks to individual drones, and generates optimized waypoint sequences for mission execution.

## Features

- **Polygon Decomposition**: Decomposes search areas into flyable cells while accounting for holes, obstacles, and UAV constraints.  
- **Coverage Path Planning**: Generates structured back-and-forth (lawnmower) coverage patterns with configurable lane width and step size.  
- **Multi-UAV Task Allocation**: Optimizes drone assignment using the Jonker–Volgenant algorithm to minimize overall mission cost.  
- **Geodetic Integration**: Converts local ENU coordinates into GPS coordinates for real-world deployment.  
- **Mission Export**: Produces QGroundControl-compatible `.plan` files for direct execution.  
- **Visualization Tools**: Visualizes area decomposition, coverage paths, and UAV assignments for validation and debugging.  
- **Modular Architecture**: Clear separation of core algorithms, I/O handling, utility functions, and test modules.


## Project Structure
```
PathPlanner/
├── __init__.py              # Package initialization and exports
├── __main__.py              # Entry point for running as a module
├── main.py                  # Main script with default test inputs
├── core/                    # Core logic
│   ├── assignment.py        # UAV-cell assignment (Jonker–Volgenant algorithm)
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
└── missions/                # Directory for generated mission files
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
   - Set the environment variable `QGC_MISSIONS_DIR` to your QGroundControl missions folder.
   **Windows (PowerShell):**
   ```powershell
   setx QGC_MISSIONS_DIR "C:\Path\To\Missions"
   ```
   **macOS/Linux (bash/zsh):**
   ```
   export QGC_MISSIONS_DIR="/path/to/Missions"
   ```

## Usage
### Basic Run

Run with default test inputs (simple rectangle, 1 UAV):
```bash
python -m PathPlanner
```
This generates paths, plots, and mission files in the configured mission directory 
(default: `PathPlanner/missions/` or the directory defined via `QGC_MISSIONS_DIR`).

### Custom Test Cases

Use predefined test cases from `tests/test_cases.py`:
```python
from tests.test_cases import test_case_graz_4uavs
from main import run_path_planner

# Load test case
config = test_case_stubaier
run_path_planner(**config)
```

### API Usage

Import and call the planner directly:

```python
from pathlib import Path
from PathPlanner.core.planners import run_path_planner
from tests.test_cases import test_case_graz_4uavs

config = test_case_graz_4uavs

run_path_planner(
    Path("missions"),
    Path("missions"),
    **config
)
```

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

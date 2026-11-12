# Golf Cart RViz2 Setup Guide

## Overview
This setup provides a clean, professional RViz2 visualization of the golf cart localization system with a 3D model representation.

## What Was Created

### 1. **Golf Cart URDF Model** (`urdf/golf_cart.urdf`)
A 3D model of the golf cart at its real size, written by `urdf/cart_model.py`
from the sizes in `urdf/cart.yaml`:
- **base_link**: the frame the localization moves (at the lidar's height, 0.5 m behind it)
- **cart_body**: the cart mesh (`urdf/golf_cart.dae`), placed so its wheels stand on the ground
- **cart_glass**: the windscreen (`urdf/golf_cart_glass.dae`), a link of its own so RViz can draw it transparent

### 2. **Clean RViz Configuration** (`rviz/localization_clean.rviz`)
A simplified visualization display with:
- **Grid**: 10×10 reference grid
- **RobotModel**: Shows the 3D golf cart in real-time
- **Map**: Reference point cloud (green points)
- **Lidar Points**: Live sensor data from Ouster (colored by ring/altitude)
- **Path**: Navigation path (green line)
- **Pose**: Current localization pose with covariance

### 3. **Updated Launch File** (`launch/lidar_localization.launch.py`)
Automatically:
- Loads the golf cart URDF
- Starts robot_state_publisher for TF broadcasting
- Launches RViz2 with the clean configuration

## Key Features

### ✓ Less Messy Display
- Removed duplicate point cloud displays
- Organized displays logically
- Color-coded for clarity:
  - **Green**: Reference map and path
  - **Multicolor**: Live lidar data (by ring number)
  - **Red/Purple**: Robot position and covariance

### ✓ 3D Golf Cart Visualization
- See the actual cart model in RViz2
- Wheels and chassis rendered in 3D
- Lidar position clearly visible on top

### ✓ Correct IMU Position
- IMU link is now at the lidar position
- TF hierarchy properly defined in URDF
- No more conflicting static transforms

## Usage

### Build the Package
```bash
cd ~/1Localize/src/lidar_localization_ros2-main
colcon build
```

### Run the Localization
```bash
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
```

This will:
1. Start the lidar localization node
2. Publish the robot state and TF transforms
3. Launch RViz2 with the clean configuration automatically

### Manual RViz2 Launch
If you only want to launch RViz2 separately:
```bash
ros2 run rviz2 rviz2 -d install/lidar_localization_ros2/share/lidar_localization_ros2/rviz/localization_clean.rviz
```

## Customization

### Modify the Cart Model
`urdf/golf_cart.urdf` and the two `.dae` files are generated; do not edit them by hand.
- Change sizes, the lidar's position on the cart or the colours in `urdf/cart.yaml`
- Change or add parts in `build_parts()` of `urdf/cart_model.py`

Then, with the localization stopped:
```bash
cd ~/2Localize_get_path
python3 src/lidar_localization_ros2-main/urdf/cart_model.py
colcon build --packages-select lidar_localization_ros2
```

### Adjust RViz Display
Edit `rviz/localization_clean.rviz` to:
- Change point cloud colors and sizes
- Add/remove displays
- Adjust camera view parameters
- Modify grid size or line widths

### Tweak Launch Parameters
Edit `launch/lidar_localization.launch.py` to:
- Load different URDF files
- Change default RViz config
- Add additional nodes or remappings

## Tips

1. **Camera Control in RViz2**
   - **Rotate**: Middle mouse button + drag
   - **Pan**: Right mouse button + drag
   - **Zoom**: Scroll wheel or Ctrl + right click drag

2. **Focus on Robot**: Use "Target Frame: base_link" in Views panel

3. **Show/Hide Elements**: Toggle displays in the left panel tree

4. **Performance**: If sluggish, reduce lidar point size or enable point decay

## Files Modified/Created
- ✅ **Created**: `urdf/golf_cart.urdf`
- ✅ **Created**: `rviz/localization_clean.rviz`
- ✅ **Modified**: `launch/lidar_localization.launch.py`
- ✅ **Modified**: `CMakeLists.txt`

## Original Configuration
The original RViz config is preserved as `rviz/localization.rviz` and can be used anytime by launching:
```bash
ros2 run rviz2 rviz2 -d install/lidar_localization_ros2/share/lidar_localization_ros2/rviz/localization.rviz
```

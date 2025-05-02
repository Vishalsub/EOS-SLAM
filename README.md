# AION-SLAM: AI-Optimized Navigation and Mapping

**AION-SLAM** is a research-oriented, modular SLAM framework that integrates deep learning and classic geometry to achieve robust, efficient, and real-time 2D/3D SLAM. Designed as a next-gen alternative to SLAM Toolbox and Cartographer, it focuses on reducing computational load while increasing performance in dynamic and large-scale environments.

## Features

- **AI-Guided ICP / Pose Refinement**: Lightweight neural modules to replace or enhance classical point cloud registration.
- **Efficient Mapping Engine**: Occupancy grid updates with learned scan fusion and map decay.
- **Learning-Based Loop Closure Detection**: CNN/RNN-based spatial memory matcher.
- **Dynamic Object Filtering**: Auto-learn to ignore moving objects (people, cars, etc.).
- **Modular ROS 2 Nodes**: Built for ROS 2 Foxy / Humble. Each major component is a separate node.

## Comparison
| Feature | SLAM Toolbox | Cartographer | AION-SLAM |
|--------|---------------|--------------|------------|
| Mapping | Pose-graph | Submap-based | Learnable hybrid |
| Loop Closure | Yes | Yes | AI-based scoring |
| Dynamic Handling | Weak | Manual config | Learnable filter |
| Compute Load | High | High | Lower via pruning and async policies |
| Adaptation | Manual tuning | YAML-based | Online self-tuning |

## Architecture
- `icp_node`: Custom-built ICP (C++)
- `map_builder_node`: Raycasting + AI-enhanced map update (OpenCV/ROS)
- `loop_closure_node`: (Optional) Deep matcher (Python/PyTorch)
- `slam_master`: ROS 2 launch orchestration

## Installation
```bash
sudo apt update
sudo apt install ros-foxy-pcl-conversions ros-foxy-tf2-tools
cd ~/ros2_ws/src
git clone https://github.com/your-username/aion-slam.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Usage
```bash
# Terminal 1: Run SLAM
ros2 run my_slam icp_node

# Terminal 2: Map Builder
ros2 run my_slam map_builder

# Optional: RViz
rviz2 -d src/my_slam/rviz/slam_config.rviz
```

## Demo
![Demo Screenshot](docs/demo.gif)

## Roadmap
- [x] ICP + Pose Estimation (C++)
- [x] Real-time Mapping (OpenCV)
- [ ] Loop Closure with Siamese Net (PyTorch)
- [ ] ROS 2 RViz Plugin
- [ ] Support for 3D SLAM (point cloud)

## Citation
Coming soon (research paper in progress).

## License
MIT License

---
Built by Vishal S., MSc Robotics @ NUS. Contributions welcome!

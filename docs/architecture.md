# EOS-SLAM Architecture

EOS-SLAM is designed for real-time SLAM by fusing stereo images, inertial measurements, LiDAR point clouds, and wheel odometry into a consistent state estimate.

## 🔄 Core Design

- **O(1) update loop**: Fixed-size sliding window optimization
- **Adaptive triggering**: Based on motion thresholds or info gain
- **Multithreaded**: ROS 2 modular nodes, asynchronous data flows
- **Modular layers**: Sensor processing, fusion core, mapping, and visualization

## 🧠 Sensor Fusion Core

| Module            | Function                           |
|------------------|------------------------------------|
| IMU Processor     | Preintegrated delta poses          |
| Stereo Depth      | Disparity → depth → 3D landmarks   |
| LiDAR Processor   | Sparse ICP scan matching           |
| Fusion Node       | Integrates all data into EKF core  |

## 🗺️ Mapping Stack

- **2D Log-Odds Grid**: Efficient occupancy update using ray casting
- **3D Local Submap**: Memory-bounded voxel representation
- **Semantic Layer (optional)**: Confidence-weighted landmarks via MobileNetV3

## 🧩 ROS 2 Integration

All modules are designed to be composable ROS 2 nodes with topic-based communication and config files in `params.yaml`.

## 📍 Output

- Full pose trajectory (TF tree)
- Occupancy grid map
- Benchmark log (latency, memory, RMSE)

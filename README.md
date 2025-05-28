# EOS-SLAM

**Event-Optimized Stereo-Inertial-LiDAR SLAM**

EOS-SLAM is a real-time hybrid SLAM system that fuses stereo cameras, IMU, LiDAR, and wheel odometry into a single, efficient architecture. Designed with a fixed-size fusion window and adaptive triggering, it delivers consistent O(1) computational performance per frame while maintaining high accuracy and robust mapping.

---

## 🚀 Features

- ✅ Real-time stereo + IMU + LiDAR fusion
- ✅ O(1) computational cost per frame
- ✅ Event-driven updates (e.g., pose delta, info gain)
- ✅ Hybrid 2D occupancy + 3D voxel map
- ✅ ROS 2 ready & modular C++ codebase
- ✅ Benchmarking support (KITTI, EuRoC, TUM)

---

## 📁 Repository Structure

- `fusion_core/`: Sensor processing & fusion logic
- `mapping/`: 2D log-odds map + 3D local voxel maps
- `localization/`: EKF / trigger manager
- `semantic/`: Optional MobileNetV3-based semantic layer
- `tools/`: Benchmark logger + RViz plugins
- `docs/`: System design & benchmarking results

---

## 📦 Datasets & Evaluation

EOS-SLAM is tested on:
- KITTI (Stereo + IMU + GPS)
- EuRoC MAV (Stereo + IMU)
- TUM RGB-D (for stereo fallback testing)

Evaluation is performed using:
- [evo](https://github.com/MichaelGrupp/evo) for ATE/RPE
- Internal logging system for CPU, FPS, and memory

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.

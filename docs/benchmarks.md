# EOS-SLAM Benchmarking

This document outlines how to benchmark EOS-SLAM against other SLAM systems.

## 📊 Metrics Used

| Metric | Tool | Description |
|--------|------|-------------|
| ATE    | evo_ape | Absolute trajectory error vs. ground truth |
| RPE    | evo_rpe | Local relative pose error                  |
| FPS    | Internal logger | Frame processing rate |
| CPU    | Internal logger | CPU % usage                         |
| Memory | Internal logger | Memory footprint per module         |

## 🧪 Datasets

- **KITTI**: Urban driving, stereo + IMU + GPS
- **EuRoC**: Indoor drone, stereo + IMU
- **TUM RGB-D**: Indoor RGB-D sequences for visual testing

## 🔧 Tools Used

- [evo](https://github.com/MichaelGrupp/evo)
- RViz for map/trajectory visualization
- Custom logger (`benchmark_logger.cpp`)

## 🧵 Procedure

1. Run EOS-SLAM with `ros2 bag play` on dataset
2. Record trajectory using ROS 2 TF or PoseStamped
3. Run ATE/RPE comparison:
   ```bash
   evo_ape tum groundtruth.txt eos_slam_trajectory.txt -a -p
---

## 📈 Target Benchmarks (Initial Goals)

EOS-SLAM aims to outperform existing SLAM systems in terms of efficiency and real-time performance while maintaining high accuracy. The following table presents our initial benchmark goals compared to industry-standard systems:

| **System**     | **ATE ↓** | **FPS ↑** | **CPU ↓** | **Memory ↓** |
|----------------|-----------|-----------|-----------|--------------|
| ORB-SLAM3      | 0.21 m    | 14 FPS    | 68%       | 1.1 GB       |
| VINS-Fusion    | 0.18 m    | 22 FPS    | 60%       | 0.9 GB       |
| **EOS-SLAM**   | **0.15 m**| **25+ FPS**| **< 50%** | **< 700 MB** |

> 📌 *Note: These benchmarks are projected goals. Actual performance will be reported after full integration and testing on KITTI, EuRoC, and TUM datasets.*

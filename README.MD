# Shobo - Multi‑Sensor ROS 2 Patrol Robot (Jetson Orin Nano)

State‑of‑the‑art, modular edge stack for **RGB + IR (IMX219) + ToF + LiDAR + Voice** with
**mid‑level fusion**, **EKF/SLAM/Nav2 patrol**, and **Qt visualization** (edge & remote).

---

## 0) Hardware

**On the way**
- **IR (NIR)**: Waveshare **IMX219‑77IR** (8MP, CSI)
- **RGB**: Arducam **IMX219** Autofocus USB (UVC)
- **Depth**: Arducam **ToF 0.43MP**
- **Voice**: Yahboom **AI Voice Recognition** module (+ speaker)
- **Compute**: **Jetson Orin Nano Super Dev Kit** (8 GB)

**Next**
- **LiDAR** (RPLIDAR A2 / YDLIDAR G4) - 2D
- **IMU** (Bosch BNO055) + **wheel encoders**
- **GNSS**: u‑blox **NEO‑M9N** (starter) or **ZED‑F9P** (RTK)

---

## 1) High‑Level Architecture

```mermaid

flowchart LR
  subgraph Sensors
    IR[IMX219-IR (CSI)] -- "/sensors/ir/image_raw" --> IRnode
    RGB[IMX219 USB (UVC)] -- "/sensors/rgb/image_raw" --> RGBnode
    ToF[ToF 0.43MP] -- "/sensors/depth/image_raw" --> TOFnode
    LIDAR[LiDAR 2D] -- "/sensors/lidar/scan" --> LIDARnode
    IMU[IMU] -- "/sensors/imu/data" --> IMUnode
    GPS[GNSS] -- "/sensors/gps/fix" --> GPSnode
    Voice[Voice Cmd] -- "/sensors/voice/command" --> VOICEnode
  end

  subgraph Sensor_Nodes
    IRnode[imx219_ir_node]
    RGBnode[uvc_rgb_node]
    TOFnode[tof_camera_node]
    LIDARnode[lidar_driver]
    IMUnode[imu_driver]
    GPSnode[gps_driver]
    VOICEnode[voice_node]
  end

  subgraph Perception
    YOLOIR[yolo_ir (TRT)]
    YOLORGB[yolo_rgb (TRT)]
    DEPTHTRK[depth_tracker]
  end

  subgraph Fusion
    FUSE[mid_fusion]
  end

  subgraph Nav
    EKF[robot_localization EKF]
    SLAM[slam_toolbox]
    NAV2[Nav2]
  end

  subgraph Viz
    QT[Qt Viewer]
  end

  IRnode --> YOLOIR
  RGBnode --> YOLORGB
  TOFnode --> DEPTHTRK
  YOLOIR --> FUSE
  YOLORGB --> FUSE
  DEPTHTRK --> FUSE

  LIDARnode --> SLAM
  IMUnode --> EKF
  GPSnode --> EKF
  EKF --> NAV2
  SLAM --> NAV2

  FUSE --> QT

```

- **Mid‑level fusion**: per‑modality **feature vectors** (RGB/IR embeddings, ToF stats, etc.) → small **MLP/attention** → `/fusion/detections`.
- **Navigation**: slam_toolbox/Cartographer + robot_localization (EKF) + Nav2 (waypoints/patrol).
- **Qt Viewer** (C++): on‑edge (DDS) or remote (rosbridge + h264/WebRTC).

---

## 2) Namespaces & Topics (SOTA)

### Sensors
- `/sensors/ir/image_raw` (`sensor_msgs/Image`) - **Waveshare IMX219‑77IR (CSI)**
- `/sensors/rgb/image_raw` (`sensor_msgs/Image`) - **Arducam IMX219 USB (UVC)`**
- `/sensors/depth/image_raw` (`sensor_msgs/Image`) - **Arducam ToF**
- `/sensors/lidar/scan` (`sensor_msgs/LaserScan`) - **RPLIDAR/YDLIDAR**
- `/sensors/imu/data` (`sensor_msgs/Imu`) - **BNO055**
- `/sensors/odom/wheel` (`nav_msgs/Odometry`) - wheel encoders
- `/sensors/gps/fix` (`sensor_msgs/NavSatFix`) - **u‑blox**
- `/sensors/voice/command` (`std_msgs/String`) - **Yahboom**

### Perception
- `/perception/yolo_rgb/detections` (`vision_msgs/Detection2DArray`) - **Arducam IMX219 USB**
- `/perception/yolo_ir/detections` (`vision_msgs/Detection2DArray`) - **Waveshare IMX219‑IR**
- `/perception/depth_tracker/objects` (`vision_msgs/Detection3DArray` or `sensor_msgs/PointCloud2`)
- (optional) `/perception/*/annotated` (`sensor_msgs/Image` via `image_transport/compressed`)

### Fusion & Nav
- `/fusion/detections` (`vision_msgs/Detection3DArray`)
- `/tf`, `/tf_static`
- `/odom` (`nav_msgs/Odometry`), `/map` (`nav_msgs/OccupancyGrid`)
- `/cmd_vel` (`geometry_msgs/Twist`) from Nav2

### Viz & Control
- `/sys/metrics` (FPS/latency/temps), `/sys/health` (diagnostics)
- `/ui/cmd` (exposure/gain, start/stop record, mission commands)

**QoS**
- Sensors: **SensorData QoS** (best_effort, depth=10-20)  
- Control/Viz: **reliable**  
- Bags/records: **reliable**, keep_last with adequate depth

---

## 3) Nodes (who does what)

| Node (package/exe) | Sensor used | Subscribes | Publishes | Notes |
|---|---|---|---|---|
| `/sensors/ir/imx219_node` | **Waveshare IMX219‑77IR (CSI)** | - | `/sensors/ir/image_raw`, `/sensors/ir/camera_info` | **Argus (NVMM zero‑copy)**, exposure/gain params |
| `/sensors/rgb/uvc_node` | **Arducam IMX219 USB** | - | `/sensors/rgb/image_raw`, `/sensors/rgb/camera_info` | `v4l2src` → NV12/BGR |
| `/sensors/depth/tof_node` | **Arducam ToF** | - | `/sensors/depth/image_raw` (+ `/points`) | vendor SDK or OpenCV |
| `/sensors/lidar/driver` | **RPLIDAR/YDLIDAR** | - | `/sensors/lidar/scan` | vendor ROS driver |
| `/sensors/imu/driver` | **BNO055** | - | `/sensors/imu/data` | i2c/spi/serial |
| `/sensors/odom/wheel_node` | Encoders | - | `/sensors/odom/wheel` | quadrature → odom |
| `/sensors/gps/ublox` | **u‑blox** | - | `/sensors/gps/fix` | ublox_ros |
| `/sensors/voice/node` | **Yahboom** | - | `/sensors/voice/command` | wake‑word & STT |
| `/perception/yolo_rgb` | **Arducam IMX219 USB** | `/sensors/rgb/image_raw` | `/perception/rgb/detections`, `/perception/rgb/annotated` | **DetectorNode** with `ModelProfile=rgb` (TRT, INT8/FP16) |
| `/perception/yolo_ir` | **Waveshare IMX219‑IR** | `/sensors/ir/image_raw` | `/perception/ir/detections`, `/perception/ir/annotated` | **DetectorNode** with `ModelProfile=ir` (diff. normalization/weights) |
| `/perception/depth_tracker` | **Arducam ToF** | `/sensors/depth/image_raw` | `/perception/depth/objects` | 3D ROIs/centroids |
| `/fusion/mid_fusion` | All | `/perception/*/detections`, `/sensors/depth/*` | `/fusion/detections` | feature‑level concat/attention; handles missing modalities |
| `/localization/ekf` | - | imu, wheel odom, gps | `/odom` | `robot_localization` EKF |
| `/slam/slam_toolbox` | **LiDAR** | `/sensors/lidar/scan` | `/map`, `/tf` | or Cartographer |
| `/nav2/*` | - | `/map`, `/odom`, `/fusion/detections` | `/cmd_vel` | waypoints/patrol |
| `/infra/rosbridge` | - | all | WS :8765 | remote API |
| `/infra/web_video_server` | - | image topics | HTTP MJPEG/H264 | remote video |
| `/viz/qt_viewer` | - | `/fusion/detections`, annotated images, `/sys/metrics` | `/ui/cmd` | runs on Orin or on another PC |


---

## 4) Mid‑Level Fusion (feature‑level)

- RGB/IR: tiny backbone embeddings (e.g., MobileNetV3‑tiny) from detector head  
- ToF: ROI depth stats, point clusters (centroid/height)  
- Concatenate or cross‑attention → small MLP → `Detection3DArray`  
- Publish in a single frame (`base_link`) using calibrated extrinsics (`/tf`)

**Sync**: to be used `message_filters::ApproximateTime` (±50 ms) for `/sensors/{rgb,ir,depth}/image_raw`.

---

## 5) Model Lifecycle (train → deploy)

**Train / finetune**
- **cloud GPU** (Colab Pro / RunPod / Lambda) or your own NVIDIA PC.
- Export to **ONNX** (`yolo_*_vX.onnx`).

**Deploy (on Orin)**
- On first run, build **TensorRT** engines (INT8/FP16) from ONNX and **cache** them:
  ```
  config/models/yolo_ir_v1/
    model.yaml        # manifest (precision, input, nms, cache dir)
    yolo_ir_v1.onnx
    calibration.cache # for INT8
  models_cache/
    yolo_ir_v1_sm87_int8_640x640.plan
  ```
- Detector nodes load from cache (hot‑reload supported), or rebuild automatically if cache missing.
- Optional when you scale: **Triton** to host multiple models; nodes become Triton clients.

---

## 6) Build & Run

### A) Sensors (example IMX219‑IR)
```bash
# Launch camera (Argus/NVMM)
ros2 launch camera_imx219 imx219.launch.py width:=960 height:=720 fps:=30   exposure_us:=80000 gain_min:=1.0 gain_max:=8.0
```

### B) Perception - YOLO IR & YOLO RGB
```bash
# Unified DetectorNode with profiles:
ros2 launch detector_node detector.launch.py   model_profile:=config/models/yolo_ir_v1/model.yaml  namespace:=/perception/yolo_ir   subscribe:=/sensors/ir/image_raw publish:=/perception/ir

ros2 launch detector_node detector.launch.py   model_profile:=config/models/yolo_rgb_v1/model.yaml namespace:=/perception/yolo_rgb   subscribe:=/sensors/rgb/image_raw publish:=/perception/rgb
```

### C) Fusion + Localization + Nav2
```bash
ros2 launch mid_fusion mid_fusion.launch.py
ros2 launch robot_localization ekf.launch.py
ros2 launch slam_toolbox online_async.launch.py
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false
```

### D) Remote visualization
```bash
# On Orin:
ros2 run rosbridge_server rosbridge_websocket --port 8765 &
ros2 run web_video_server web_video_server &
```
**On your PC** (Qt app container or native) point to:
- WebSocket: `ws://<orin-ip>:8765`  
- Video (example): `http://<orin-ip>:8080/stream?topic=/perception/ir/annotated&type=ros_compressed`

**Best performance**: run Qt on the **Orin** for DDS zero‑copy.  
**Flexible remote**: Qt on **PC** via rosbridge + compressed video (or WebRTC).

---

## 7) Performance, QoS & Observability
- Enable **intra‑process comms** / component containers for zero‑copy between nodes on Orin.
- Prefer `image_transport/compressed` for WAN/remote viewing.
- Benchmark mode: `sudo nvpmodel -m 0 && sudo jetson_clocks`.
- Export `/sys/metrics` (FPS, latency, GPU temp, mem) and show in Qt.
- Record with `rosbag2` (**MCAP + zstd**, split by size/time).

---

## 8) Project Layout 

```
mmx/
├─ sensors/           # camera_imx219, uvc_rgb, tof, lidar, imu, gps, voice
├─ perception/        # detector_node (profiled), depth_tracker
├─ fusion/            # mid_fusion
├─ localization/      # robot_localization (ekf), tf trees
├─ slam/              # slam_toolbox or cartographer
├─ nav2/              # behavior trees, patrol waypoints
├─ viz/qt/            # Qt C++ app (edge/remote)
├─ infra/             # rosbridge, web_video_server
├─ config/
│   ├─ camera_imx219.yaml
│   ├─ detector_rgb.yaml / detector_ir.yaml
│   ├─ fusion.yaml
│   ├─ ekf.yaml, nav2.yaml, slam.yaml
│   └─ models/
│       ├─ yolo_ir_v1/{model.yaml, yolo_ir_v1.onnx, calibration.cache}
│       └─ yolo_rgb_v1/{...}
└─ models_cache/
```

---

## 9) Security (remote ops)
- Put rosbridge/web_video behind **WireGuard**/**Tailscale**.
- Or enable **SROS2** (DDS security) for encrypted topics.

---

## 10) License
TBD

---

## 11) Roadmap
- [ ] IMX219 node ✓
- [ ] DetectorNode (TRT) with profiles for RGB/IR
- [ ] Mid‑fusion (MLP/attention)
- [ ] EKF + SLAM + Nav2 patrol pipeline
- [ ] Qt edge + Qt remote + metrics
- [ ] Model A/B, hot reload, rollback

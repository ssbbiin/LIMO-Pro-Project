# LIMO Pro Autonomous Navigation & Sensor Fusion

> ROS 2 기반 RGB-D SLAM, Localization 및 자율주행 시스템 구축과  
> Wheel Odometry / Wheel+IMU EKF Sensor Fusion 비교 실험

## Project Overview

본 프로젝트에서는 LIMO Pro 모바일 로봇을 이용하여 ROS 2 기반 실내 자율주행 시스템을 구축하고,
센서융합 방식에 따른 자율주행 성능 차이를 실험적으로 비교하였다.

Orbbec RGB-D Camera와 RTAB-Map을 이용하여 실내 지도를 생성하고,
생성된 지도를 기반으로 Localization 및 Nav2 자율주행 환경을 구성하였다.

이후 위치추정 방식을 다음 두 조건으로 분리하였다.

- Wheel Odometry only
- Wheel Odometry + IMU → EKF Sensor Fusion

동일한 지도와 Nav2 설정을 유지한 상태에서 반복 자율주행 실험을 수행하고,
주행 성공률, Navigation Time, Recovery 횟수, 경로 오차,
최종 위치 및 자세 오차 등을 분석하였다.

## Tech Stack

| Category | Technology |
|---|---|
| Robot | LIMO Pro |
| OS | Ubuntu 22.04.5 LTS |
| Middleware | ROS 2 Humble |
| RGB-D Camera | Orbbec RGB-D Camera |
| SLAM | RTAB-Map |
| Localization | RTAB-Map Localization |
| Navigation | Nav2 |
| Sensor Fusion | robot_localization EKF |
| Sensors | Wheel Odometry, IMU, RGB-D Camera, LiDAR |
| Data Logging | rosbag2 |
| Analysis | Python |

## System Architecture

<p align="center">
  <img src="system_architecture.png" width="100%">
</p>

Wheel Odometry와 IMU를 EKF로 융합하여 `/odometry/filtered`를 생성하고,
RGB-D Camera와 융합 Odometry를 RTAB-Map의 입력으로 사용하여 Mapping 및 Localization 환경을 구성하였다.
Nav2는 생성된 지도와 위치추정 정보, LiDAR `/scan` 기반 장애물 정보를 이용하여 경로 계획 및 자율주행을 수행한다.

## Implementation

### 1. Sensor & TF Configuration

LIMO Pro에 탑재된 RGB-D Camera, Wheel Odometry, IMU 및 LiDAR의 ROS 2 토픽을 확인하고,
각 센서 데이터의 발행 주기와 좌표계(Frame)를 분석하였다.

주요 센서 토픽은 다음과 같다.

| Sensor | ROS 2 Topic | Description |
|---|---|---|
| RGB Camera | `/camera/color/image_raw` | RGB Image |
| Depth Camera | `/camera/depth/image_raw` | Depth Image |
| Wheel Odometry | `/odom` | Wheel Odometry |
| IMU | `/imu/data` | Orientation / Angular Velocity / Acceleration |
| Fused Odometry | `/odometry/filtered` | EKF-based Odometry |
| LiDAR | `/scan` | 2D LaserScan |

RGB-D Camera의 Depth 데이터를 Color Camera 좌표계에 정렬하고,
`base_link`를 기준으로 카메라의 실제 장착 위치와 각도를 반영하도록 TF를 구성하였다.


<p align="center">
  <img src="rtab1.png">
</p>

<p align="center">
  <em>RGB-D Camera와 Odometry를 이용한 RTAB-Map 기반 3D Mapping 결과</em>
</p>

### 3. Localization & Nav2 Autonomous Navigation

Mapping을 통해 생성한 지도를 기반으로 RTAB-Map을 Localization mode로 실행하고,
Nav2와 연동하여 실내 자율주행 환경을 구성하였다.

RTAB-Map은 RGB-D Camera 데이터와 Odometry를 이용하여 지도 상에서 로봇의 위치를 추정하며,
Nav2는 현재 위치와 사용자가 지정한 Goal Pose를 기반으로 Global Path와 Local Path를 생성한다.

주행 중에는 LiDAR `/scan` 데이터를 Nav2 Costmap의 Obstacle Layer에 입력하여
주변 장애물을 반영하도록 구성하였다.

이를 통해 동일한 지도와 Navigation 설정을 유지한 상태에서
위치추정 방식에 따른 자율주행 성능을 비교할 수 있는 실험 환경을 구축하였다.
<p align="center">
  <img src="nav2_start.png" width="850">
</p>

<p align="center">
  <em>LIMO-PRO 시작지점</em>
</p>

### 4. Wheel Odometry Only — Baseline

센서융합의 효과를 비교하기 위한 Baseline으로 Wheel Odometry만을 이용하여
로봇의 이동 상태를 추정하는 조건을 구성하였다.

이 조건에서는 IMU 데이터를 위치추정에 사용하지 않고,
Wheel Encoder로부터 계산된 Odometry를 기반으로 RTAB-Map Localization과
Nav2 자율주행을 수행하였다.

Wheel Odometry는 바퀴의 회전량을 기반으로 로봇의 이동량을 추정하기 때문에
주행 거리가 증가하거나 회전 및 미끄러짐이 발생할 경우 오차가 누적될 수 있다.

이후 Wheel Odometry + IMU EKF 조건과 동일한 지도, 시작 위치, Goal Pose 및
Nav2 설정에서 반복 주행하여 자율주행 성능을 비교하였다.


### 5. Wheel Odometry + IMU EKF Sensor Fusion

Wheel Odometry의 누적 오차를 보완하기 위해 IMU 데이터를 함께 사용하고,
ROS 2 `robot_localization` 패키지의 Extended Kalman Filter(EKF)를 이용하여
센서융합 기반 Odometry를 구성하였다.

EKF는 Wheel Odometry와 IMU의 측정값을 융합하여 `/odometry/filtered`를 생성하며,
이를 RTAB-Map Localization 및 Navigation을 위한 Odometry 정보로 사용하였다.

이를 통해 Wheel Odometry만 사용하는 Baseline과 비교하여 IMU 융합이
로봇의 위치 및 자세 추정과 실제 자율주행 성능에 미치는 영향을 분석하였다.

#### Odometry Configuration

| Configuration | Input | Odometry used for Localization / Navigation |
|---|---|---|
| **Wheel Odometry Only (Baseline)** | Wheel Encoder | `/odom` |
| **Wheel + IMU EKF (Sensor Fusion)** | Wheel Odometry + IMU | `/odometry/filtered` |

## Experimental Setup

Wheel Odometry Only와 Wheel Odometry + IMU EKF의 차이가 실제 자율주행 성능에
미치는 영향을 비교하기 위해 동일한 주행 환경에서 반복 실험을 구성하였다.

두 조건에서 Mapping 결과와 Nav2 설정은 동일하게 유지하고,
Odometry 구성만 변경하여 위치추정 방식에 따른 성능 차이를 비교하였다.

### Controlled Conditions

| Parameter | Condition |
|---|---|
| Map | Same RTAB-Map |
| Start Pose | Same start position and orientation |
| Goal Pose | Same target position and orientation |
| Navigation | Same Nav2 configuration |
| Environment | Same indoor test environment |
| Variable | Odometry configuration |

### Evaluation Metrics

| Metric | Description |
|---|---|
| Navigation Success | Goal 도달 성공 여부 |
| Navigation Time | 출발부터 Goal 도달까지 소요 시간 |
| Recovery Count | 주행 중 발생한 Recovery 횟수 |
| Path Deviation | 기준 경로 대비 실제 주행 경로의 편차 |
| Final Position Error | Goal Pose와 최종 위치 사이의 오차 |
| Final Yaw Error | 목표 자세와 최종 자세 사이의 각도 오차 |
### Navigation Result Comparison

동일한 Start Pose와 Goal Pose를 설정하고 두 Odometry 조건에서 반복 자율주행을 수행하였다.
아래는 Wheel Odometry Only와 Wheel Odometry + IMU EKF 조건에서의
Nav2 자율주행 결과를 비교한 RViz 화면이다.

| Wheel Odometry Only | Wheel Odometry + IMU EKF |
|---|---|
| <img src="wheel_only_goal.png" width="400"> | <img src="sensor_fusion_goal.png" width="400"> |
| **Baseline** | **Sensor Fusion** |

## Results & Analysis

복잡한 장애물 환경에서 Wheel Odometry Only와 Wheel + IMU EKF 조건을 비교하였다.

Goal A와 Goal B를 각각 5회씩 수행하여 모드당 10회,
총 20회의 자율주행 데이터를 수집하였다.

### Quantitative Results

| Metric | Wheel Odometry Only | Wheel + IMU EKF |
|---|---:|---:|
| Success Rate | 90% (9/10) | **100% (10/10)** |
| Avg. Navigation Time | 45.14 s | **19.22 s** |
| Median Navigation Time | 24.80 s | **17.95 s** |
| Avg. Recovery Count | 0.8 | **0.0** |
| Avg. Trajectory Length | 4.404 m | **4.248 m** |
| Path MAE | **0.0239 m** | 0.0269 m |
| Path RMSE | **0.0326 m** | 0.0338 m |
| Path Max Error | 0.1312 m | **0.0960 m** |
| Final Position Error | 0.2470 m | **0.2129 m** |
| Final Yaw Error | **10.90°** | 11.67° |
### Key Performance Comparison

<p align="center">
  <img src="avg_navigation_time.png" width="32%">
  <img src="success_rate.png" width="32%">
  <img src="avg_recoveries.png" width="32%">
</p>

##  Results & Observations

동일한 Mapping 결과와 Nav2 설정에서 Wheel Odometry Only와
Wheel Odometry + IMU EKF 조건으로 반복 자율주행 실험을 수행하였다.

실험 과정에서 주행 시간, Recovery 발생 여부, 경로 추종 특성,
Goal 도달 상태 등을 기록하였으며, 일부 Wheel Odometry 조건에서는
주행 중 일시적인 정체 및 위치추정 불안정 현상이 관찰되었다.

반면 센서융합 조건에서도 모든 상황에서 성능이 일관되게 향상되는 것은
아니었으며, 주행 환경과 위치추정 상태에 따라 차이가 발생하였다.

이를 통해 센서 추가 자체보다 센서 데이터의 특성, EKF 설정 및
Localization과의 상호작용이 실제 자율주행 성능에 중요함을 확인하였다.

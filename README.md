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

RGB-D Camera와 Odometry 정보를 기반으로 RTAB-Map에서 Mapping 및 Localization을 수행하고,
Wheel Odometry와 IMU는 EKF를 통해 융합하여 위치추정에 활용하였다.
Nav2는 지도 및 위치추정 정보와 LiDAR 기반 장애물 정보를 이용하여 경로 계획 및 자율주행을 수행한다.

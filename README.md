# central-computer

## Programs

### `marker_detection`
- Transforms an angled view to a birds eye view
- Extracts ID and pose from ArUco markers

### `command_transmission`

### `multi_trajectory_planner`

### `local_planner`
- A stripped nav2_controller program running Regulated Pure Pursuit

## Hardware
- ESP32 MCUs
- Faulhaber 2224R006SR 054 motors
    - Encoders: IE2-512
    - [Additional information](https://chatgpt.com/share/690e30bb-ef28-8010-8c69-22c305963a47)
- Motor drivers 
    - Find a model that supports the Faulhaber motor power requirements

## Optimizations
Several optimizations could be made to benefit the performance of the system. Some of these optimizations include:
- Rewriting Python programs in C++

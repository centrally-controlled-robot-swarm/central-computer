# central-computer

## Programs

### `marker_detection`
- Transforms an angled view to a birds eye view
- Extracts ID and pose from ArUco markers
- `pip install opencv-contrib-python`
- Calibration done by the [camera_calibration library](https://github.com/abhishekpadalkar/camera_calibration)

### `command_transmission`
- Facilitates UDP transmission between a laptop hot spot and each MCU
- 💡 An example is currently provided in the program source code

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
- Regenerate a global plan at set intervals
- Calibrate with charuco markers

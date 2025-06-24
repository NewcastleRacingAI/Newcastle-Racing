# Changelog
All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.0] - 2023-01-30
### Added
- Simulated bounding boxes plugin
- Simulated camera cones plugin 
- Noisy cone colors in simulated perception
- Robot selection in launcher GUI
- Track refresh button in launcher GUI
- Stop ground truth pub button in launcher GUI
- New tracks
- Question issue template
- IMU for ZED Camera

### Changed
- New ADS-DV mesh
- Sensor positions for the ADS-DV robot
- Simulated perception topic to `/cones` from `/fusion/cones`
- `base_link` renamed to `chassis`
- Topic prefix for sensors
- Revise track generation
- Launcher module structure
- Robot Steering GUI defaults to `/cmd` topic
- Replace the `/ros_can/set_mission` topic with an equivalent service
- Reduce artifact overhead in CI pipeline

### Fixed
- Robot model no longer "sinks" in RVIZ
- `base_footprint` position adjusted to ground level
- Add missing dependencies in eufs_sensors
- Transform frame for the depth camera
- setup.cfg deprecation warnings
- Removed x velocity clipping in vehicle model

## [2.0.0] - 2021-11-16
### Added
- Allow selection of different vehicle model parameter configurations
- Create a GUI button to reset simulation environment
- Add ground truth and noisy wheel speeds topics
- Support both velocity and acceleration commands
- Implement steering rate limit
- Add control delays
### Changed
- Migrate from ROS 1 to ROS 2 Galactic
- Reduce simulator visual complexity
- Extract vehicle model as a C++ library
- Split the launcher and track generator
- Migrate non-track launch files to python
- Scale cone noise with distance from car
- Improve directory structure
- Improve documentation
- Get ROS message times directly from Gazebo
### Deprecated
- Disable launcher and track generator noise sliders
### Removed
- VLP-16 sensor URDF and meshes
- GPS with mast sensor URDF and meshes
- Hokuyo UTM-30LX sensor URDF and meshes
- IMU Hector Plugin sensor URDF and meshes
- Kinect sensor URDF and meshes
- Kinect v2 sensor URDF and meshes
- Point Grey Bumblebee 2 sensor URDF and meshes
- Wheelchair robot URDF and meshes
### Fixed
- Fix missing package dependencies
- Fix linting errors
- Once in emergency state, only legal option should be reset
- Remove ghost cones in RViz
- Stop listening to driving flag for AS transition
- AMI state should indicate manual driving
- EBS not activated properly when triggered from AS_READY
- Robot steering GUI is not reactive to the command mode of vehicle model
- Remove noise from /ground_truth/state
- Catch InvalidTopicError in robot steering GUI when topics end in "/"

## [1.0.0] - 2020-09-01
### Added
- EUFS Launcher which is a GUI that allows you to customise the launching of the simulation
- State machine according to the Formula Student rules
- Mission selector and steering RQT GUI
- Race car model plugin to make the car behave according to the dynamic bicycle vehicle model
- Cone ground truth plugin to publish cone locations and simulate perception
- Includes plugins for various sensors including IMU, GPS, VLP LiDAR, and ZED camera
- Preset track layouts including tracks for acceleration and skidpad
- Random track generator

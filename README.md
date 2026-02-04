# Newcastle Racing 2025/2026

# WARNING THIS IS A TEMPORARY BRANCH TO ATTEMPT TO GET A SIMPLE UNIVERSAL WORKING CONFIG FOR ROS2 ON DOCKER   

To start working on this project, clone the respository and ensure you are on the right branch.

```bash
git clone --recurse-submodules https://github.com/NewcastleRacingAI/Newcastle-Racing2526.git
cd Newcastle-Racing2526
git checkout rowan
```

to update your local git repo
```bash
git pull && git submodule update --init --recursive
```

## Setting up a development enviroment

We are currently suggesting configureing a docker volume mount to share a portion of your host filesystem with the nrai docker container, you can then build, run and test your program within that container.   

To configure your development environment navigate to the docker-compose.yml file and uncomment the lines for `volume` and its corresponding pathreplacing `your-project` with the name you want for the directory that your code resides in AND replacing `/host/system/path` with the path to the directory your code resides in on your host system. **YOU MUST RETAIN THE COLON BETWEEN THE PATHS**


## Running via Docker

**You will first need to install docker on your system**

First run
```bash
docker build -t rowan-nrai:latest .
```

The full stack (with some limitations) can be launched as a Docker container.
To do so, simply use the command

```bash
docker compose up
```

The first time, the image will have to be pulled from the registry, which will take some time.
All subsequent times, after a few seconds, open your browser and navigate to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) and click connect.
You should see a small screen which will allow you to launch the simulator.

## Useful commands

get a bash command line on a container
```bash
docker exec -it nrai bash
```
(replace nrai for a different contatiner)

rebuild and relaunch
```bash
docker build -t rowan-nrai:latest . &&
docker compose up --detach
```
(must be ran in the projects root dir as . is shorthand for that)

stop docker containers
```bash
docker compose down
```

## Troubleshooting

You may find that you need to run docker build within WSL ([windows subsystem for linux](https://learn.microsoft.com/en-us/windows/wsl/install)) as we have found that some windows programs can alter line endings in a way that will break scripts. If you are suffering from this you will likely see \\r in an error message. When building from inside WSL you should start with a fresh clone of the repo to ensure that you receive a correct file. The linux distribution you install should not matter but ubuntu is probably a safe bet.

## Advanced

### Direct mode (Linux only)

If you are on Linux and using X11, you can avoid having to use your browser, launching all GUI application natively on your desktop instead.
The commands to run are

```bash
xhost local:root
docker compose -f docker-compose.direct.yml up
```

After a few seconds, the launcher window should appear on your screen.



## Project structure

This is the main workspace repository containing all the sub-packages that make up the Newcastle Racing AI project.

```bash
Newcastle-Racing
├── src # Ros packages
│   ├── zed-ros2-wrapper # Ready
│   ├── eufs_msgs # Ready
│   ├── ros_can # Ready
│   ├── nrai_odometry # TODO
│   ├── nrai_perception # TODO
│   ├── nrai_path_planning # 1  - TODO
│   ├── ft-fsd-path-planning # 1 
│   ├── nrai_controller # TODO
│   └── nrai_mission_control # TODO
└── hello_world.sh # Entry point script
```


## Launching the ZED camera

To enable custom object detection for the zed camera, change the following:

```yaml
# common_stereo.yaml

# ...
        object_detection:
            od_enabled: true # True to enable Object Detection
            enable_tracking: true # Whether the object detection system includes object tracking capabilities across a sequence of images.
            detection_model: 'CUSTOM_YOLOLIKE_BOX_OBJECTS' # 'MULTI_CLASS_BOX_FAST', 'MULTI_CLASS_BOX_MEDIUM', 'MULTI_CLASS_BOX_ACCURATE', 'PERSON_HEAD_BOX_FAST', 'PERSON_HEAD_BOX_ACCURATE', 'CUSTOM_YOLOLIKE_BOX_OBJECTS'
            max_range: 20.0 # [m] Upper depth range for detections.The value cannot be greater than 'depth.max_depth'
            filtering_mode: 'NMS3D' # Filtering mode that should be applied to raw detections: 'NONE', 'NMS3D', 'NMS3D_PER_CLASS'
            prediction_timeout: 2.0 # During this time [sec], the object will have OK state even if it is not detected. Set this parameter to 0 to disable SDK predictions
            allow_reduced_precision_inference: false # Allow inference to run at a lower precision to improve runtime and memory usage
            # Other parameters are defined in the 'object_detection.yaml' and 'custom_object_detection.yaml' files
# ...
```

```yaml
# Custom 

# ...
/**:
  ros__parameters:
      object_detection:
          custom_onnx_file: '.../yolo11s.onnx' # Path to the YOLO-like ONNX file for custom object detection directly performed by the ZED SDK
          custom_onnx_input_size: 672 # Resolution used with the YOLO-like ONNX file. For example, 512 means a input tensor '1x3x512x512' 
          
          custom_class_count: 5 # Number of classes in the custom ONNX file. For example, 80 for YOLOv8 trained on COCO dataset

          # TODO: Add one instance of each class to the list below
          # Note: create a class_XXX identifier for each class in the custom ONNX file.
          # Note: XXX is a number from 000 to 'custom_class_count-1', and it must be unique for each class.
          # Note: the class_XXX identifier is not required to match the class ID [model_class_id] in the custom ONNX file.

          class_000:
            label: 'blue_cone'
            model_class_id: 0 # Class ID of the object in the custom ONNX file (it is not required that this value matches the value in the 'class_XXX' identifier)
            enabled: true # Enable/disable the detection of this class
            confidence_threshold: 50.0 # Minimum value of the detection confidence of an object [0,99]
            is_grounded: true # Provide hypothesis about the object movements (degrees of freedom or DoF) to improve the object tracking
            is_static: true # Provide hypothesis about the object staticity to improve the object tracking
            tracking_timeout: -1.0 # Maximum tracking time threshold (in seconds) before dropping the tracked object when unseen for this amount of time
            tracking_max_dist: -1.0 # Maximum tracking distance threshold (in meters) before dropping the tracked object when unseen for this amount of meters. Only valid for static object
            max_box_width_normalized: -1.0 # Maximum allowed width normalized to the image size
            min_box_width_normalized: -1.0 # Minimum allowed width normalized to the image size
            max_box_height_normalized: -1.0 # Maximum allowed height normalized to the image size
            min_box_height_normalized: -1.0 # Minimum allowed height normalized to the image size
            max_box_width_meters: -1.0 # Maximum allowed 3D width
            min_box_width_meters: -1.0 # Minimum allowed 3D width
            max_box_height_meters: -1.0 # Maximum allowed 3D height
            min_box_height_meters: -1.0 # Minimum allowed 3D height
            object_acceleration_preset: 'DEFAULT' # Object acceleration preset. Possible values: 'DEFAULT', 'LOW', 'MEDIUM', 'HIGH'
            max_allowed_acceleration: 100000.0 # If set with a different value from the default [100000], this value takes precedence over the selected preset, allowing for a custom maximum acceleration. Unit is m/s^2.
# ...
```

```bash
# Make sure both the global and local setup are sourced
source /opt/ros/humble/setup.bash && source install/setup.bash 
# Launch the zed camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i custom_object_detection_config_path:=.../src/nrai_perception/resource/custom_object_detection.yaml
# Optionally, launch the rviz2 tool for a visualization of the camera topics
# Note that you will need to build the zed_display_rviz2 ROS package
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
```

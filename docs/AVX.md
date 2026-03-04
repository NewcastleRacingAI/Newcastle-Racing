# AVX API Python Samples

This folder includes the below-listed Python scripts, as well as some related resource files, intended to provide a simplified workflow to use the 2025 R2 Ansys AVxcelerate Sensors Simulator APIs.

- `grpc_channel.py`: illustrates how to configure and open the gRPC channel with the default transport mode (TLS) and with UDS or Insecure. For more information about transport modes, refer to the specifications document for your software version.
- `simulation_control.py`: illustrates how to control a simulation sequence through AVxcelerate Sensors Simulator APIs
- `sensor_data_retrieval.py`: illustrates how to access sensors' output data through the gRPC Data Access server
- `sensor_data_retrieval_shm.py`: illustrates how to access sensors' output data from shared memory
- `camera_feedback_control.py`: illustrates how to update the focal length and exposure time of a camera sensor during a simulation
- `radar_feedback_control.py`: illustrates how to toggle ON/OFF radar mode during a simulation
- `parse_sensors_output_protobuf_file.py`: illustrates how to load a protobuf dump file into a protobuf message

Before using those sample scripts, install the [Prerequisites](#Prerequisites), then go through the steps in the [Deploying](#Deploying) section.

The _AvxApiPython_ folder can be copied and deployed anywhere on your hard disk, so it is safe to add your custom script.
The path to the project folder is therefore denoted as `<myProjectPath>` in this document.

## <a id="Prerequisites">Prerequisites</a>

### Software installations

- Python >= 3.8.0 (Dependency packages have been tested with Python 3.8.0 and 3.10.10)
- Ansys AVxcelerate Sensors Simulator 2025 R2
- Ansys AVxcelerate Sensors Standalone Library 2025 R2

Make sure that you have at least the following two folders under the AVX installation directory (the installation directory is _%AWP_ROOT252%/Autonomy/AVxcelerateSensors_ on Windows and the installation directory is _/ansys_inc/v252/Autonomy/AVxcelerateSensors_ on Linux):

- _VSS_
- _APIs_

### Third party libraries

These samples rely on several libraries which will be automatically downloaded when installing the Python dependency packages [on Windows](#InstallingPythonWindows) or [on Linux](#InstallingPythonLinux).

## <a id="Deploying">Deploying</a>

**Note**: In the commands provided in these procedures, the following file paths need to be replaced:

- \<myProjectPath\> with the path to your project folder
- \<myPythonRootPath\> with the path to your Python root

### Deploying on Windows

1. Edit your 'Path' environment variable to add the Python root path and the path to the _Scripts_ subfolder.  In the below paths, **XXX** corresponds to the Python version (e.g. for Python 3.10 **XXX** must be replaced with `310`).

    - _%LocalAppData%\Programs\Python\Python**XXX**_
    - _%LocalAppData%\Programs\Python\Python**XXX**\Scripts_

2. (optional) We recommend you work from a new Python virtual environment. To do so:

    - install virtualenv package to your Python root.

        ```powershell
        cd <myPythonRootPath>
        python -m pip install virtualenv
        ```

    - Come back to your project folder and create a new local Python virtual environment:

        ```powershell
        cd <myProjectPath> ## For example "$env:AWP_ROOT252/Autonomy/AVxcelerateSensors/APIs/VSS_API/Samples/AvxApiPython"
        virtualenv -p python3 .venv
        ```

    - Activate this new venv:

        ```powershell
        ./.venv/Scripts/activate
        ```

        Once the Python virtual environment is activated, your command line starts with `(.venv) PS>`.

3. Update and <a id="InstallingPythonWindows">install the Python dependency packages</a>:

    ```powershell
    (.venv) PS> python -m pip install --upgrade pip
    (.venv) PS> python -m pip install -r requirements.txt
    ```

4. Generate the protobuf, gRPC and Python files from .protoc:

    - Open the file _make_proto.bat_ and edit the first line with your custom AVX installation path (the default installation directory is _C:\Program Files\ANSYS Inc\v252\Autonomy\AVxcelerateSensors_).
    - Generate *.py files from .protoc file.

        ```powershell
        (.venv) PS>./make_proto.bat
        ```

    **Note**: The grpcio-tools package must be installed in your current Python environment.
5. Verify that the sample is correctly deployed by running the _simulation_control.py_ script.

### Deploying on Linux

1. Install Python3 if it is not done:

    ```console
    sudo apt-get update -y
    sudo apt-get install python3 -y
    sudo apt-get install python3-pip -y
    ```

2. It is recommended to work from a new Python virtual environment. To do so:

    - If the virtualenv package is not pre-installed on your root, you can manually install it:

        ```console
        sudo apt-get install python3-venv
        ```

    - Come back to your project folder and create a new local Python virtual environment:

        ```console
        cd <myProjectPath> ##For example "$env:AWP_ROOT252/Autonomy/AVxcelerateSensors/APIs/VSS_API/Samples/AvxApiPython"
        python3 -m venv .venv
        ```

    - Activate this new venv:

        ```console
        source ./.venv/bin/activate
        ```

       Once the Python virtual environment is activated, your command line starts with `(.venv) PS>`.

3. From inside the virtual environment, update and <a id="InstallingPythonLinux">install the Python dependency packages</a>:

    ```console
    (.venv)$ pip3 install --upgrade pip
    (.venv)$ pip3 install -r ./requirements.txt
    ```

4. Generate the protobuf, gRPC and Python files from .protoc:

    - Open the file _make_proto.sh_ and edit the first line with your custom AVX installation path (the default installation directory is _/ansys_inc/v252/Autonomy/AVxcelerateSensors/APIs/VSS_API/avx_).
    - Generate *.py files from .protoc file.

        ```console
        (.venv)$ . ./make_proto.sh
        ```

    **Note**: The grpcio-tools package must be installed in your current Python environment.

5. Verify that the sample is correctly deployed by running the _simulation_control.py_ script.

## <a id="tls">TLS (mTLS) configuration</a>

By default, the samples use **mTLS** transport mode. Before running them:

1. **Prepare certificates:** required files are `server.crt`, `server.key`, `client.crt`, `client.key`, `ca.crt` (see the specifications document).
2. **Point the to the certificates folder** by setting the environment variable `ANSYS_GRPC_CERTIFICATES` on the machine:

   **Windows (PowerShell)**

   ```powershell
   $env:ANSYS_GRPC_CERTIFICATES="C:\path\to\certs"

   ```

   **Linux (bash)**

   ```bash
   export ANSYS_GRPC_CERTIFICATES=/home/user/certs
   ```

**Note**: If you want to use **UDS** or **Insecure** transport mode instead, change `DEFAULT_TRANSPORT_MODE` in `grpc_channel.py`. For more details on transport modes, see the specifications document.

⚠️ **Warning:** Insecure transport mode usage is not recommended.

## Running the samples

### Running simulation_control.py

To run this sample, just launch the Python script and have a look at the console.

The script launches AVxcelerate Sensors Simulator, runs a simulation with a sample scenario (from the _trajectory_city.json_ file), and then kills AVxcelerate Sensors Simulator at the end of the simulation.
This script illustrates all the simulation commands and machine states in a valid sequence.

```
$ python simulation_control.py

    =============================================================================================================
    This Python script illustrates how to control an AVX simulation. It shows a valid simulation sequence
    with the following simulation commands:
        LOAD -> INIT -> n UPDATE -> STOP -> UNLOAD -> KILL
    - AVX Sensors Simulator will be launched in a separate thread and will be killed at the end of the simulation.
    - A sample scenario is loaded from the trajectory_city.json file.
    =============================================================================================================

[...]

Received message after RUNNING simulation step 1: Successfully executed UpdateCommand.
Received message after RUNNING simulation step 2: Successfully executed UpdateCommand.

[...]

Received message after RUNNING simulation step 27: Successfully executed UpdateCommand.
```

### Running sensor_data_retrieval.py

Before running this sample:

- launch an AVX simulation including at least one sensor from any driving simulator (for example CarMaker) with the -d simulation process argument followed by the port to use for communication over gRPC, and the -p simulation process argument followed by port 54321.
- Make sure that the sample and AVxcelerate Sensors Simulator are configured to use the same transport mode.

Then, launch the Python script.

The console displays the list of sensors included in the simulation as well as information on each sensor's output data.

```
$ python ./sensor_data_retrieval.py

    ==============================================================================================================
    This Python script illustrates how to access sensors output data through gRPC Data access server.
    - This code should be run asynchronously with AVxcelerate Sensors Simulator.
    - You must activate the gRPC Data Access Server with the -d <port> argument.
    - Press Ctrl + C to exit.
    ==============================================================================================================

========================================================
-- Data time stamp: seconds: 5
nanos: 680000000

-- Sensor ID: camera
Data output is not split
RAW image received with size: (720, 1280, 4)
========================================================
-- Data time stamp: seconds: 5
nanos: 720000000

-- Sensor ID: camera
Data output is not split
RAW image received with size: (720, 1280, 4)
========================================================
```

If you want to filter the received data based on sensor IDs, launch the Python script with the `--sensorsIds` option followed by a list of sensor IDs separated by a space character.
For example:

```
$ python ./sensor_data_retrieval.py --sensorsIds front_camera roof_lidar
```

**Note:** The sample and AVxcelerate Sensors Simulator must use the same transport mode. If you change `DEFAULT_TRANSPORT_MODE` in `grpc_channel.py`, launch AVxcelerate Sensors Simulator or the driving simulator with the matching mode.

### Running sensor_data_retrieval_shm.py

Before running this sample:

- launch an AVX simulation including at least one sensor from any driving simulator (for example CarMaker) with the -p simulation process argument followed by port 54321.
- Make sure that the sample and AVxcelerate Sensors Simulator are configured to use the same transport mode.

Then, launch the Python script.

The console displays the list of sensors included in the simulation as well as information on each sensor's output data.

```
$ python ./sensor_data_retrieval_shm.py

    ==============================================================================================================
    This Python script illustrates how to access sensor data directly from the shared memory.
    - This code should be run asynchronously with AVxcelerate Sensors Simulator on the same host.
    - Press Ctrl + C to exit.
    ==============================================================================================================

========================================================
-- Data time stamp: seconds: 7
nanos: 360000000

-- Sensor ID: camera
Data output is not split
RAW image received with size: (720, 1280, 4)
========================================================
-- Data time stamp: seconds: 7
nanos: 400000000

-- Sensor ID: camera
Data output is not split
RAW image received with size: (720, 1280, 4)
========================================================
```

If you want to filter the received data based on sensor IDs, launch the Python script with the `--sensorsIds` option followed by a list of sensor IDs separated by a space character.
For example:

```
$ python ./sensor_data_retrieval_shm.py --sensorsIds front_camera roof_lidar
```

### Running camera_feedback_control.py

Before running this sample:

- launch an AVX simulation including at least one sensor from any driving simulator (for example CarMaker) with the -fbc simulation process argument to activate the Feedback Control service, and the -p simulation process argument followed by port 54321.
- Make sure that the sample and AVxcelerate Sensors Simulator are configured to use the same transport mode.

Then, launch the Python script.

Some camera parameters are updated during the simulation.

```
$ python ./camera_feedback_control.py

    ==========================================================================================
    This Python script illustrates how to update camera parameters using AVX API.
    - This code should be run asynchronously with AVxcelerate Sensors Simulator.
    - You must activate the Feedback Control service with the -fbc|--feedbackcontrol argument.
    ==========================================================================================

----------------------------------------
camera's parameters updated to:
- exposure_time  = 5e-06 s
- focal_length   = 0.003
----------------------------------------
camera's parameters updated to:
- exposure_time  = 2.5e-05 s
- focal_length   = 0.003
----------------------------------------
```

### Running radar_feedback_control.py

Before running this sample:

- launch an AVX simulation including at least one sensor from any driving simulator (for example CarMaker) with the -fbc simulation process argument to activate the Feedback Control service, and the -p simulation process argument followed by port 54321.
- Make sure that the sample and AVxcelerate Sensors Simulator are configured to use the same transport mode.

Then, launch the Python script.

```
$ python radar_feedback_control.py

    ==========================================================================================
    This Python script illustrates how to use the feedback control to switch on and off
    one or several radar modes using AVX API.
    - This code should be run in parallel, asynchronously with AVxcelerate Sensors Simulator.
    - You must activate the Feedback Control service with the -fbc|--feedbackcontrol agrument.
    ==========================================================================================

Turn OFF mode 0
--Feedback control message:
sensor_id: "radar"
feedback_control_radar_parameters {
  modes {
    identifier {
    }
    activate_mode {
    }
  }
}

Turn ON mode 0
--Feedback control message:
sensor_id: "radar"
feedback_control_radar_parameters {
  modes {
    identifier {
    }
    activate_mode {
      value: true
    }
  }
}

```

### Running parse_sensors_output_protobuf_file.py

To run this sample, just launch the Python script.

The script parses the following three protobuf dump files provided in the _data_ folder:

- a radar range doppler map: _radar_rdm/radar_000001000_radar.pb.bin_
- a LiDAR contribution list: _lidar_rotating/lidar_rotating_000001000_contributions.pb.bin_
- a LiDAR point cloud: _lidar_rotating/lidar_rotating_000001000_pointcloud.pb.bin_

The script provides useful information on the data according to the type of output.

```
$ python parse_sensors_output_protobuf_file.py

    ============================================================================
    This script illustrates how to load a protobuf dump file into a protobuf message.
    ============================================================================

Radar range doppler file loaded.
- Range domain info:
size: 600
max_value: 150.24737548828125

- velocity domain info:
size: 400
min_value: -53.985694885253906
max_value: 43.74076843261719

Lidar contribution list file loaded. Number of values in the contribution list:         64000
Lidar point cloud file loaded. Number of points in the point cloud:         64000
```
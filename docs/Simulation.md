# Simulation

## Ansys


### Installation steps

1. Download the Ansys installer from the [official website](https://download.ansys.com/currentReleases) and launch it.
  Note that the installer expects to be run as root, if you can't, you may need to go through steps 2-3.
1. Since the installation is graphical, you may need to give the root user access to the display
    ```bash
    xhost si:localuser:root
    ```
1. If you are using firefox from snap, you may run into an issue where, during the installation process, the installer tries to load a webpage but firefox does not find it, despite the file existing.
    The way we solved this was to install [Chrome](https://www.google.com/intl/en_uk/chrome/dr/download/) on the machine and set it as the default browser for root.
    ```bash
    sudo xdg-settings set default-web-browser google-chrome.desktop
    ```
    Note that to run Chrome as root, the `--no-sandbox` flag must be added when the program is invoked.
    A simple workaround to obtain this result is to edit the Chrome startup script.
    Substitute `$EDITOR` with your favourite (e.g., `code`, `nano`, `vim`)
    ```bash
    $EDITOR $(which google-chrome)
    ```
    and change the last line by adding `--no-sandbox`.
1. After that, the installation process can begin:
    Dowload the AnsysInstaller.sh and run it with sudo, and then follow the installation steps.
    ```bash
    sudo ./AnsysInstaller.sh
    ```
    In our usecase, only the **AVX** utilities are needed, the rest can be ignored.
    At the end of the installation, if you have not changed the default paths, you should find a folder `/ansys_inc` which contains all the relevant files.
1. It seems that the **LicenceManager** does not get installed properly via the simple installation script.
    Instead, it has to be downloaded separately from the [official website](https://download.ansys.com/currentReleases).
    The compressed folder (something like `ANSYSLICMAN_2025R2.04_LINX64.tgz`) uncompresed and the installation performed with
    ```bash
    sudo ./INSTALL -LM
    ```
    Note that you may need to repeat the setup steps 2-3 to allow the root user to successfully install the **LicenceManager**.
1. The licence manager should start automatically, and it can always be started manually with
    ```bash
    sudo /ansys_inc/shared_files/licensing/tools/bin/linx64/ansyslmcenter
    ```
    On the [webpage](http://localhost:1084/ANSYSLMCenter.html), go in the `Get System Hostid Information` section, take note of the `hostname` and `hostid` and use that information to generate a licence.
    The licence file must then be loaded in the `Add license file` section.
    Ensure that everything is running from the `View Status/Start/Stop License Manager` section.  
    Note that you will need to start the server every time you reboot your system, unless you add it to the list of systemd services.
1. Once the **LicenceManager** is running and the license setup, you need to make sure the **LincenceSettings** are pointing to the licence server.
    To do this, launch the **LicenceSetting** utility
    ```bash
    sudo /ansys_inc/v252/licensingclient/linx64/LicensingSettings
    ```
    In the **LicenceManager** [webpage](http://localhost:1084/ANSYSLMCenter.html), in the _View FlexNex Debug Log_ section, take note of the port being used (it should be port `1055`).
    Use this information to fill the `FlexiNet Publisher > License Servers` section.
    | Port | Server 1  | Server 2 | Server 3 |
    | ---- | --------- | -------- | -------- |
    | 1055 | localhost |          |          |

    Test the connection to ensure the server is available and save your changes.
1. It would be wise to revent the changes made in steps 2-3
     ```bash
     xhost -SI:localuser:root
     ```

### Testing the installation

1. Go to `/ansys_inc/v252/Autonomy/AVxcelerateSensors/APIs/VSS_API/Samples/AvxApiPython`.
1. Install the python depencencies in `requirements.txt` (either globally or in a virtual environmnet).
    ```bash
    python3 -m pip install -r requirements.txt
    ```
1. Run the setup script
    ```bash
    make_proto.sh
    ```
1. **(WORKAROUND to the certificates problem)** Modify the `grpc_channel.py` substituting the line
    ```py
    DEFAULT_TRANSPORT_MODE = TransportMode.MTransportLayerSecurity
    ```
    with
    ```py
    DEFAULT_TRANSPORT_MODE = TransportMode.Insecure
    ```
1. Launch the example script 
    ```bash
    python3 simulation_control.py 
    ```
    You should see something like this:
    ```bash
        =============================================================================================================
        This Python script illustrates how to control an AVX simulation. It shows a valid simulation sequence 
        with the following simulation commands:
            LOAD -> INIT -> n UPDATE -> STOP -> UNLOAD -> KILL
        - AVX Sensors Simulator will be launched in a separate thread and will be killed at the end of the simulation.
        - A sample scenario is loaded from the trajectory_city.json file.
        =============================================================================================================
        
    2026-02-13 13:30:58,765 [1] INFO  Vss.Pre.Vss.Program [(null)] - Starting AVxcelerate Sensors Simulator.
    2026-02-13 13:31:00,698 [5] INFO  Vss.Inf.Actors.Private.Communication.Remote [(null)] - Starting remote server on 127.0.0.1:54322...
    2026-02-13 13:31:01,804 [13] WARN  Vss.Ccc.GrpcNetRemote.Private.RpcServer [(null)] - WARNING: Insecure transport is enabled, it is not the recommended configuration.
    2026-02-13 13:31:01,894 [13] INFO  Vss.App.SimulationInterface.Private.Services.SimulationInterfaceService [(null)] - AVX Sensors server server is started on 127.0.0.1:54321.
    2026-02-13 13:31:06,675 [7] INFO  Vss.App.Topolyzer.Private.TopolyzerActivity [(null)] - Entering Started state
    2026-02-13 13:31:07,149 [18] INFO  Vss.Engine.Inf.LoaderService.Private.LoaderService [(null)] - Loading track /tmp/n1hww5zp.b1u/Track/city.env...
    2026-02-13 13:31:07,633 [18] INFO  Vss.Engine.Inf.LoaderService.Private.LoaderService [(null)] - Loading assets...
    2026-02-13 13:31:09,156 [18] INFO  Vss.Engine.Inf.EgoVehicleService.Private.EgoVehicleService [(null)] - The sensor will be loaded relatively to the 'SensorReferential'.
    2026-02-13 13:31:09,156 [18] INFO  Vss.Engine.Inf.LoaderService.Private.LoaderService [(null)] - Track and assets successfully loaded
    2026-02-13 13:31:29,520 [18] INFO  Vss.App.SimulationInterface.Private.Services.CommandExecutionService [(null)] - Load completed. Switching to Loaded state
    2026-02-13 13:32:01,222 [19] INFO  Vss.App.SimulationInterface.Private.Services.CommandExecutionService [(null)] - Init completed. Switching to Running state
    Received message after RUNNING simulation step 1: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 2: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 3: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 4: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 5: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 6: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 7: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 8: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 9: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 10: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 11: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 12: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 13: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 14: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 15: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 16: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 17: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 18: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 19: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 20: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 21: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 22: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 23: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 24: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 25: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 26: Successfully executed UpdateCommand.
    Received message after RUNNING simulation step 27: Successfully executed UpdateCommand.
    2026-02-13 13:32:19,258 [35] INFO  Vss.App.SimulationInterface.Private.SimulationInterfaceActivity [(null)] - The simulation interface server has been shut down.
    2026-02-13 13:32:19,259 [33] INFO  Vss.App.Orchestrator.Private.Services.LocalProcessDataAccessService [(null)] - Closing DataAccess Server
    2026-02-13 13:32:19,285 [33] INFO  Vss.Inf.Actors.Private.Communication.Remote [(null)] - Shutting down Remote...
    2026-02-13 13:32:19,288 [42] INFO  Vss.Inf.Actors.Private.Communication.Remote [(null)] - Remote shut down.
    ```

## Starting CarMaker via AVX

- Move to the [/home/newcastleracing/CM_Projects/AVX_SENSORS/2025R2/CM13/UI](/home/newcastleracing/CM_Projects/AVX_SENSORS/2025R2/CM13/UI) and launch the `StartAVXConnectorGUI_13.0.sh` script.
You may need to change the content of the script to make sure it points to the correct path to the CarMaker executable.

```sh
# E.g.,
cd /home/newcastleracing/CM_Projects/AVX_SENSORS/2025R2/CM13/UI
./StartAVXConnectorGUI_13.0.sh
```

## Folders

- Installation folder [/ansys_inc](/ansys_inc)
- Car Maker Projects folder [/home/newcastleracing/CM_Projects/AVX_SENSORS/2025R2/CM13](/home/newcastleracing/CM_Projects/AVX_SENSORS/2025R2/CM13)
- Library [/opt/Ansys/AVX_Library_v252/](/opt/Ansys/AVX_Library_v252/)

## Configuration paths

- AVX executable = `/usr/ansys_inc/v252/Autonomy/AVxcelerateSensors/VSS/avxcelerate.sensorssimulator`
- AVX executable args = `--port 54321 -fbc -lsc -d 54545 -h 127.0.0.1 -t insecure -o  /home/newcastleracing/Documents/AVX_Out`
- Deploy paramters = `/opt/CM_Projects/AVX_SENSORS/2025R2/CM13/Data/Config/deploy-parameters.json`
- Simulation parameters = `/opt/Ansys/AVX_Library_v252/FS-UK/resources/ZedCamera_Lidar_SimulationParameters_DisplayOnly.json`
- Cosim map = `/opt/Ansys/AVX_Library_v252/FS-UK/02_Assets Mapping/CarMakerAssetsMapping_NaturalSky.json`
- Sensor config = `/opt/Ansys/AVX_Library_v252/FS-UK/Configurations/FS_AI_1PBCam_Zed_0.92MP_original_VLP32_10Hz_Lidar.sencfgx`

## CarMaker

### Installation

Register on the [IPG Formula Student](https://www.ipg-automotive.com/support/licenses/formula-carmaker) website.
You may need to prove you are part of a team enrolled in the Formula Student competition.
After a while, you should receive confirmation you have been accepted and, with that, access to the [cutomer area](https://www.ipg-automotive.com/support/customer-area) of the website.
From there, you can download CarMaker for your platform of choice and **"LTS for FS-AI 2026"**, a zip file containing the project to load into CarMaker.

Extract the folder and check that the cone `.node` files are present.
They should be in `MovieNX/data/TrafficCones`.
If they are not there, try copying them over from an older version of **"LTS for FS-AI 2026"** (if you have access to any).
Otherwise, the cones will not appear.

Then, the setup begins.
- Source ros `source /opt/ros/humble/setup.bash`
- Make all bash scripts that you will need executable: `chmod +x build.sh ros/ros2_ws/build.sh CMStart.sh`
- Update the symlink
    ```bash
    cd lib
    sudo ln -sfn libcmcppifloader-linux64.so.1.0.0 libcmcppifloader-linux64.so
    ```
- Build CarMaker `./build.sh`
- Run CarMaker `./CMStart.sh`
- From the GUI:
  - `Application > IO configuration` add a virtual bus interface, select vCAN for the `vBus 0`, and enable all things that can be enabled.
    Save and make sure this configuration is used by the simulator.
  - `Application > Configuration / Status` add `-io can` to the `Command line options`
  - `Extra > CMRosIF > Launch & start Application` to launch the ROS bridge
  - `File > Movie NX` to launch the visualization tool
  - `File > Open` to select your test run

Finally, you can start your simulation.

### Added June 2025:
# Running Docker Containerised Code

Note: the docker implementation was created on a linux ubuntu 22.04 pc. It may be able to be run on other systems but some modification may be required.

The expected workflow is that all members of the team are able to build the docker container for testing modified src code and commit everything back to git. Changes to src will require rebuilding inside the container. 

1. run
```
xhost +local:root
docker-compose up
```
when finished it will say "Attaching to nufs", run in separate command line:
```
docker exec -it nufs bash
```

2. Inside the container run the MPCC install file:
```
cd src/MPCC/C++
rm -rf External/
./install.sh
cd /workspace/src/ft-fsd-path-planning
pip install --no-cache-dir -r requirements.txt
```

3. Build ROS packages
```
cd /workspace
source /opt/ros/galactic/setup.bash
pip3 install --no-cache-dir pandas matplotlib scipy
rosdep install --from-paths src --ignore-src -r -y
```
ignore errors about pandas, matplolib and scipy
```
colcon build --symlink-install
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

When running new nodes, open a new command shell and run the following:
```
docker exec -it nufs bash
cd /workspace
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

### Running the Simulation

see section "Launching and Running Nodes" below. 
```
ros2 launch eufs_launcher eufs_launcher.launch.py
./install/ft-fsd-path-planning/bin/path_planning_node
ros2 run mpcc_control mpcc_control_node
```

### Shutting down the container
To save container as is a new image:
```
docker commit nufs your_custom_name
```

To end the current container session but persist all changes to be ran again:
```
exit
```

And to start that version of the container again:
```
docker start -ai nufs
cd /workspace
source /opt/ros/galactic/setup.bash
source install/setup.bash
```

# For Non-dockerised use of the code:

### Initial Setup and Environment Configuration

1. **Navigate to Workspace:**
   Start by changing to your ROS2 Foxy workspace directory located in your home folder. This is where all your development and builds will take place. This workspace name can be anything, just make sure you replace it in the commands accordingly. For example if my workspace was `eufs` I would `cd /eufs`.
   ```bash
   cd ~/your_workspace
   ```colcon build --symlink-install --packages-select ft-fsd-path-planning

2. **Edit some files in MPCC:**
   From your workspace, enter MPCC and then C++, and modify `install.sh` with `gedit`, `nano`, `vim`, or your preferred editor to add a build target for `CMake` so it won't fail on certain systems. Then, run the installer to get all the dependencies. These will automatically build within the package.
   ```bash
   cd ~/your_workspace/MPCC/C++
   gedit install.sh
   ```
   Ensure the last section of the file looks like this:
   ```bash
   cd External/blasfeo
   mkdir -p build
   mkdir -p lib
   cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../lib) -DTARGET=X64_AUTOMATIC
   make
   make install

   cd ../../hpipm
   mkdir -p build
   mkdir -p lib
   cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=$(realpath ../lib) -DBLASFEO_PATH=$(realpath ../../blasfeo/lib) -DTARGET=X64_AUTOMATIC
   make
   make install
   ```
   The main part added here is `-DTARGET=X64_AUTOMATIC`. You can replace this with another build type if you like but it will always default to `GENERIC` if it can't build on your system with this flag and is safe and verified working.

   After modifying, save and exit your editor and run:
   ```bash
   ./install.sh
   ```
   This should build without error, else `colcon build` will fail in your workspace.

> [!IMPORTANT]
> You don't need to update the `mpcc_controller.cpp` under `~/your_workspace/MPCC/C++` as the path is now dynamic and not hard-coded (see code below for quick reference) - I kept the code block to avoid confusion

   ```bash
   cd ~/your_workspace/MPCC/C++
   gedit mpcc_controller.cpp
   ```
   Near the top of the file:
   
   ```cpp
   std::string getCurrentWorkingDirectory() {
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != NULL) {
        return std::string(cwd);
    } else {
        perror("getcwd() error");
        return "";
    }
   }

   std::string getParentPath(const std::string& path) {
      std::size_t pos = path.find_last_of('/');
      if (pos != std::string::npos) {
         std::size_t pos2 = path.find_last_of('/', pos - 1);
         if (pos2 != std::string::npos) {
               return path.substr(0, pos2);
         }
      }
      return "";
   }

   class MPCCController : public rclcpp::Node {
   public:
      MPCCController()
         : Node("mpcc_controller_node"),
         last_steering_angle(0.0),colcon build --symlink-install --packages-select ft-fsd-path-planning
         track(getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/track.json"), 
         total_distance(0.0), 
         D(0.5),
         path_direction(0.0, 0.0){
         last_position = Eigen::Vector2d(0.0, 0.0); 
         initializeSubscribersAndPublishers();
      }

   private:
      void initializeSubscribersAndPublishers() {
         auto jsonConfig = loadConfig(getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/config.json");

         // 参数初始化
         mpc = std::make_unique<mpcc::MPC>(jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], jsonConfig["Ts"], loadJsonPaths());
         
         path_subscriber = this->create_subscription<eufs_msgs::msg::PathWithBoundaries>(
               "/path", 1, std::bind(&MPCCController::path_callback, this, std::placeholders::_1));
         car_state_subscriber = this->create_subscription<eufs_msgs::msg::CarState>(
               "/odometry_integration/car_state", 1, std::bind(&MPCCController::car_state_callback, this, std::placeholders::_1));
         control_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
      }

      json loadConfig(const std::string& filename) {
         std::ifstream iConfig(filename);
         if (!iConfig.is_open()) {
               RCLCPP_FATAL(this->get_logger(), "Unable to open configuration file: %s", filename.c_str());
               throw std::runtime_error("Unable to open configuration file.");
         }
         json j;
         try {
               iConfig >> j;
         } catch (json::parse_error& e) {
               RCLCPP_FATAL(this->get_logger(), "JSON parse error in configuration file: %s", e.what());
               throw;
         }
         return j;
      }

      mpcc::PathToJson loadJsonPaths() {
         return {
               getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/model.json",
               getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/cost.json",
               getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/bounds.json",
               getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/track.json",
               getParentPath(getCurrentWorkingDirectory())+"/install/mpcc_control/share/mpcc_control/Params/normalization.json"
         };
      }
   ```

1. **Install requirements in `ft-fsd-path-planning`:**
   Go back to your workspace root and access the `ft-fsd-path-planning` directory. Trying to build this directory with the original instructions given on their GitHub will cause a plethora of errors. Simply install the Python requirements from the `requirements.txt` and this should be enough for the package to build and work correctly.

   ```bash
   cd ~/your_workspace/ft-fsd-path-planning
   pip install -r requirements.txt
   ```

   Note, Linux might warn you of certain routes not being in your PATH, simply add them and it should be okay. Some good Linux packages to have or ensure are installed:

   ```bash
   sudo apt install python-is-python3
   sudo apt install python3-pip
   python -m pip install -U pip
   ```

   Recognize the Python command as Python 3, make sure PIP is installed and upgrade it. Your system default version on Ubuntu 20.04 should be `Python 3.8.*`, just use this.

2. **Build Packages:**
   After installing dependencies in `MPCC` and `ft-fsd-path-planning`, compile all the available packages in your workspace. This ensures that any changes or updates to the packages are built. You DO NOT have to clone EUFS separately, it is included in this repo along with `eufs_msgs` and `eufs_rviz_plugins`. Before building your workspace, you need to initialize, update, and install `rosdeps`. Make sure your ROS and Rosdep packages are working fine.

   ```bash
   source /opt/ros/foxy/setup.bash
   ros2 wtf  # Runs a doctor script to check install
   rosdep  # Will print an error with a list of commands
   ```

   After verifying, add the path to your workspace to PATH as an environment variable `$EUFS_MASTER`:

   ```bash
   echo 'export EUFS_MASTER=/path/to/your_workspace' >> ~/.bashrc
   source ~/.bashrc
   echo $EUFS_MASTER  # Verify path
   ```

   ```bash
   sudo rosdep init
   rosdep update --include-eol-distros
   rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y --rosdistro foxy
   ```

   It should tell you all rosdeps have been installed successfully. If you get an error saying the definition for `ament_python` cannot be found, ignore it. It doesn't seem to affect anything. In your workspace's root directory:

   ```bash
   colcon build  --symlink-install
   ```

   Make sure every package has built successfully.

3. **Environment Setup:**
   Load the ROS2 Foxy setup configurations to set up the necessary environment for ROS2 operations. You can add most of these long commands to bash aliases in your `.bashrc` to spare you having to type them frequently.
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   If you are in a ROS2 Galactic environment, you may need to change the file setup.py, CMakeList, package.xml in MPCC AND ft-fsd-path-planning.Not tested
    ```bash
    source /opt/ros/galactic/setup.bash
    ```
4. **Local Environment Configuration:**
   After building the packages, source the local setup script to update your shell with the environment settings of the newly built packages. 
   ```bash
   source install/local_setup.bash
   ```

### Launching and Running Nodes

Each node has to be launched in a separate terminal. Ensure you source `ros` and `eufs` in each new terminal window before running a node.

5. **Launch Simulation:**
   Execute the EUFS launcher to start the simulation environment, which is crucial for testing in a controlled virtual setting.
   ```bash
   ros2 launch eufs_launcher eufs_launcher.launch.py
   ```

6. **Execute Path Planning Node:**
   Run the path planning node which is part of the ft-fsd-path-planning package. This node is responsible for calculating and managing the driving path.
   ```bash
   ./install/ft-fsd-path-planning/bin/path_planning_node
   ```

7. **Execute MPCC Control Node:**
   Start the MPCC (Model Predictive Control and Contours) control node to manage vehicle dynamics and controls within the simulation.
   ```bash
   ros2 run mpcc_control mpcc_control_node
   ```

### Code Modifications and Re-deployment

8. **Selective Package Rebuilding:**
   If there are modifications in the path planning code, rebuild just the ft-fsd-path-planning package using symlink installation to quickly apply changes without affecting other packages.
   ```bash
   colcon build --packages-select ft-fsd-path-planning --symlink-install
   ```

9. **Rebuild Control Package:**
   Similarly, if changes are made to the MPCC control package, selectively rebuild it to ensure that all updates are compiled and ready for deployment.
   ```bash
   colcon build --packages-select mpcc_control
   ```

------------------------------------------------------------

colcon build // Build all packages in the ROS2 workspace

source /opt/ros/foxy/setup.bash // Source the setup file for ROS2 Foxy to configure the environment

source install/local_setup.bash // Source the local setup file to configure the environment for built packages

ros2 launch eufs_launcher eufs_launcher.launch.py // Launch the simulation with the EUFS launcher

./install/ft-fsd-path-planning/bin/path_planning_node // Execute the path planning node

ros2 run mpcc_control mpcc_control_node // Execute the MPCC control node


colcon build --packages-select ft-fsd-path-planning --symlink-install // Build only the path planning package with symlink installation

colcon build --packages-select mpcc_control // Build only the MPCC control package

--------------------------------------------------------------------------------------------------

ros2 topic list // List all ROS2 topics

ros2 topic info /camera_1/cones // Get information about the /camera_1/cones topic

ros2 topic echo /camera_1/cones // Display messages from the /camera_1/cones topic

ros2 topic echo /path // Display messages from the /path topic

ros2 topic info /camera/imu/data // Get information about the /camera/imu/data topic

ros2 topic echo /camera/imu/data // Display messages from the /camera/imu/data topic

ros2 topic echo /camera/imu/data // Repeat displaying messages from the /camera/imu/data topic

ros2 topic echo /odometry_integration/car_state // Display messages from the /odometry_integration/car_state topic

colcon build --packages-select mpcc_control // Rebuild the MPCC control package

ros2 topic echo /imu/data // Display messages from the /imu/data topic

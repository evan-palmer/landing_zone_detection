## Landing Zone Detection ROS Package

### Table of Contents
* [Summary](#summary)
* [Installation](#installation)
  * [Raspberry Pi 4 Setup](#setup-instructions-for-ros-noetic-and-intel-realsense-on-the-raspberry-pi-4)
  * [Increase Swap Size](#increase-swap-size)
  * [ROS Installation](#ros-installation)
  * [Intel RealSense SDK Installation](#installing-the-intel-realsense-sdk)
  * [RealSense ROS Package Installation](#install-the-realsense-ros-package)
  * [Landing Zone Detection Package Installation](#install-the-landing-zone-detection-package)
* [Usage](#usage)
  * [General Usage](#general-usage)
  * [Manipulating the Camera Parameters](#manipulating-the-camera-parameters)

***

### Summary
The landing zone detection package contains the implementation for a ROS package responsible for collecting depth images from the Intel Realsense D455i camera. The implementation has functionality implemented to convert a ROS Image message to an OpenCV matrix, and evaluate this matrix to determine the field-of-view and to determine the maximum gradient in the image. Additional functionality has been provided to visualize and interact with the depth image in an OpenCV window. The directory stucture for the Landing Zone Package is provided below. Here, one may see that the source code is located in the `src/` directory. The launch file is located within the `launch/` directory.

```bash
├── landing_zone_detection/
│   ├── launch/
│   │   └── landing_zone_launch.launch
│   ├── src/
│   │   └── landing_zone_detector.cpp
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── params.yaml
└── └── README.md
```

***

### Installation
The following instructions serve to document how to install the full system.

#### Setup Instructions for ROS Noetic and Intel RealSense on the Raspberry Pi 4
The following instructions include the steps necessary to install ROS Noetic and the Intel RealSense SDK onto the Raspberry Pi 4 (RPi4). These instructions assume
that the RPi4 has Ubuntu 20.04 (NOT 20.10) Desktop installed.

**NOTE:** At the moment, ROS Noetic does not support Ubuntu 20.10, only Ubuntu 20.04. You can find the previous releases of Ubuntu here.

#### Increase Swap Size
Prior to beginning, it is recommended that the swap size of the RPi4 is increased from 100 MB to 1024 MB. To do this, follow the instructions below.


1. Install `dphys-swapfile`
    * `sudo apt install dphys-swapfile`
2. Turn off swap memory
    * `sudo dphys-swapfile swapoff`
3. Modify the swap file to increase the swap space
    * `sudo nano /etc/dphys-swapfile`
    * Change the line `CONF_SWAPSIZE=2`, to `CONF_SWAPSIZE=1024`
4. Reconfigure the swap size using the updated file
    * `sudo dphys-swapfile setup`
5. Turn swap back on
    * `sudo dphys-swapfile swapon`


#### ROS installation
1. Configure your Ubuntu repositories to allow `restricted`, `universe`, and `multiverse`
    * `sudo add-apt-repository universe`
    * `sudo add-apt-repository multiverse`
    * `sudo add-apt-repository restricted`
2. Setup your computer to accept software from packages.ros.org.
    * `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
3. Setup your keys.
    * `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C`
4. Update your packages
    * `sudo apt update`
5. Install the ROS header build dependencies
    * `sudo apt-get install -y python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall build-essential cmake`
6. Initialize `rosdep`
    * `sudo rosdep init`
    * `rosdep update`
7. Create a new ROS workspace
    * **NOTE:** You may change the workspace name to your preferred naming.
    * `mkdir ~/ros_catkin_ws`
    * `cd ~/ros_catkin_ws`
8. Generate a list of Noetic dependencies to install
    * `rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall`
    * **NOTE:** There should be no output if the command is run successfully
9. Fetch all of the remote repos and install them to the `src` folder
  * `wstool init src noetic-ros_comm-wet.rosinstall`
10. Install the system dependencies
    * `rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=ubuntu:focal`
11. Compile the Noetic packages
    * **NOTE:** After you run this command, go take a walk, this will take a while to finish
    * `sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python`
12. Set up your `.bashrc` file to overlay the ROS environment on top of your existing environment.
    * `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`
    * `source .bashrc`
13. Verify the installation by running the command `roscore`.


#### Installing the Intel RealSense SDK
1. Ensure that the Intel RealSense Camera is NOT connected to the RPi4
2. Download the RealSense SDK installation script
    * `wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh`
3. Modify the permissions of the installation script
    * `chmod +x ./libuvc_installation.sh`
4. Execute the build script
    * **NOTE:** If you didn't take a walk earlier, don't worry. You can now because this command will also take some time to finish.
    * `./libuvc_installation.sh`
5. Verify that the installation was successful
    * Connect your Intel RealSense Camera to the RPi
    * Execute the command: `rs-enumerate-devices`

#### Install the RealSense ROS Package
1. Navigate to your ROS workspace's `src` directory
    * `cd ros_catkin_ws/src`
2. Clone the latest Intel RealSense ROS wrapper package
    * ```
      git clone https://github.com/IntelRealSense/realsense-ros.git
      cd realsense-ros/
      git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1\`
      cd ..
      ```
3. Build the Realsense ROS Package
    * Ensure that you are in the `ros_catkin_ws/src` directory
    * `catkin_init_workspace`
    * `cd..`
    * `catkin_make clean`
    * `catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release`
    * `catkin_make install`
    * **NOTE:** When building the package, you may run into missing package errors. When this occurs, install the missing package and rebuild the package.
4. Export the development environment to your existing environment
    * `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
    * `source ~/.bashrc`

#### Install the Landing Zone Detection Package
1. Navigate to your ROS workspace's `src` directory
    * `cd ros_catkin_ws/src`
2. Install the Landing Zone Detection package
    * `git clone git@github.com:evan-palmer/landing_zone_detection.git`
3. Build the Landing Zone Detection package
    * `cd..`
    * `catkin_make`

#### Installation Check
After installation completion, the system should have the following directory structure.
```bash
├── ros_catkin_ws/
│   ├── build/
│   ├── devel/
│   ├── install/
│   ├── src/
│   │   ├── landing_zone_detection/
└── └── └── realsense_ros/
```

***

### Usage
The following instructions serve to document how to utilize the implemented Landing Zone Detection system.

#### General Usage
1. Navigate to the ROS workspace
    * `cd ros_catkin_ws`
2. Launch the Realsense ROS package and the Landing Zone Detection package from the Landing Zone Detection launch file
    * `roslaunch landing_zone_detection landing_zone_launch.launch`
    * This should launch the both of the specified packages
    * **NOTE:** If the OpenCV window has been enabled in the implementation, the system should begin to display the depth images received. The depth images will depict the depth mapping provided by the Intel Realsense camera. On the window, a box has been drawn to depict the invalid depth band that is computed. To get see the distance at a particular pixel in the depth image, hover over the pixel, or click the pixel in the OpenCV window. This will print out the depth, in centimeters, to the terminal.

#### Manipulating the Camera Parameters
To adjust the parameters of the Intel Realsense camera, follow the instructions below.
1. Navigate to the Landing Zone Detection package launch file
    * `cd ros_catkin_ws/src/landing_zone_detection/launch/`
2. Open the launch file using your file editor of choice
    * `nano landing_zone_launch.launch`
3. In the launch file one RealSense filter is enabled: the `temporal` filter. This is observable in the following line. To integrate additional filters, add them to this line by specifying them in a comma-delimited fashion (e.g., `"temporal,hole_filling"`).
    * `<arg name="filters" value="temporal"/>`
4. In the launch file, several RealSense camera parameters are set. These values can be adjusted to their desired settings.
    * `enable_auto_exposure` 
        * Value: `False`
    * `exposure` 
        * Value: `20000`
    * `frames_queue_size` 
        * Value: `16`
    * `filter_smooth_delta` 
        * Value: `30`
    * `filter_smooth_alpha` 
        * Value: `0.2000000059604645`


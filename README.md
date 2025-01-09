# Robotics Foundations Lab Materials - 2025

## Getting Ready

### Install Ubuntu 22.04

You can either install ubuntu locally or install it on a USB. For this, follow the instructions given on the following links:

- In a PC: [https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- In a USB: [https://www.youtube.com/watch?v=j2RYqahtkNc](https://www.youtube.com/watch?v=j2RYqahtkNc)

### Windows Subsystem Linux (WSL)

It is possible to run the simulation and code for RF labs in a WSL image. For this make sure to have installed WSL () and that you have admin rights in your PC. Then, you can either download Ubuntu 22.04 from the Microsft Store: [https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=GB](https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=GB) or run the following command in a command prompt:

```bash
wsl --install Ubuntu-22.04
```

The installer will ask you for a username, make sure you use 'user' as username. It will then ask you for a password, this can be whatever you prefer. After this, you will have a working Ubuntu distro running in Windows.

## In a Ubuntu 22.04 installation (WSL or Local Installation)

Open a linux terminal and issue the following commands:

*NOTE: if you are using WSL, you will get a terminal ready after the installation ends.*

```bash
sudo apt update
sudo apt upgrade -y
```

### ROS2 Install

You now can install ROS2 (Humble) by following the instructions in this link: [CLICK HERE](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

After installing ROS, run the following commands, each at a time:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
echo "export export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
````

### Simulation and RF packages

The next step is to get and build the simulator and related ROS packages for RF. So, run (one line at a time):

```bash
sudo apt update
sudo apt install ros-humble-libfranka -y
```

```bash
sudo apt install -y ros-humble-ament-cmake ros-humble-ament-cmake-clang-format ros-humble-angles ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-ros2-control-test-assets ros-humble-controller-manager ros-humble-control-msgs ros-humble-control-toolbox ros-humble-generate-parameter-library ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-moveit ros-humble-pinocchio ros-humble-realtime-tools ros-humble-xacro ros-humble-hardware-interface ros-humble-ros-gz python3-colcon-common-extensions ros-$ROS_DISTRO-rmw-cyclonedds-cpp python3--ipykernel python3-jupyter-client
```

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

```bash
mkdir -p ~/franka_ros2_ws/src
```

```bash
source ~/.bashrc
```

```bash
cd ~/franka_ros2_ws/src
```

```bash
git clone https://github.com/gerac83/rf_simulator.git
```

```bash
cd ~/franka_ros2_ws
```

```bash
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

```bash
source install/setup.sh
```

Only run the following command if you are running WSL with a GPU. For other installation types, this is not needed, just make sure to have your drivers up-to-date.

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
```

```bash
echo "export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/home/${USER}/franka_ros2_ws/src/rf_simulator/" >> ~/.bashrc
```

```bash
source ~/.bashrc
```

```bash
cd ~/franka_ros2_ws/src
```

The following command assumes that you have VS Code installed in your Windows environment or in your Ubuntu base installation, if not make sure to download from: [https://code.visualstudio.com/](https://code.visualstudio.com/)

```bash
code .
```

### Running the Simulation and Motion Planning

Open two terminals (same for VS Code) and issue the following commands in both terminals:

```bash
cd ~/franka_ros2_ws
source install/setup.sh
```

Everything is now set up so you should be able to run the following commands, start one in each terminal:

1. `ros2 launch franka_gazebo_bringup gazebo_empty.launch.py`
2. `ros2 launch franka_gazebo_bringup moveit_sim.launch.py`

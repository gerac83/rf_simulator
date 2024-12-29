# Robotics Foundations Lab Materials - 2025

## TODO:

- [X] Test RF Materials in WSL and write instructions
- [ ] Test pymoveit2 and run examples https://github.com/AndrejOrsula/pymoveit2/tree/master
- [X] Test this repo in the constructsim https://app.theconstruct.ai/rosjects/my_rosjects/
- [X] Reduce meshes for arm and gripper models to improve simulation
- [X] Add instructions for Ubuntu installation (USB pendrive or local installation)
- [ ] Upgrade RF Lab 1
- [ ] Deploy and test RF Lab 1 in constructsim
- [ ] Upgrade RF Lab 2
- [ ] Deploy and test RF Lab 2 in constructsim
- [ ] Upgrade RF Lab 3
- [ ] Deploy and test RF Lab 3 in constructsim
- [ ] Upgrade RF Lab 4
- [ ] Deploy and test RF Lab 4 in constructsim
- [ ] Upgrade RF Lab 5
- [ ] Deploy and test RF Lab 5 in constructsim
- [ ] Design and prepare Coursework
- [ ] FUTURE: Try to compile moveit2 from source in humble to have python enabled
- [ ] FUTURE: Port RF Materials to Gazebo Garden (or wait for Franka to do it)
- [ ] FUTURE: Check https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html

## Getting Ready 

### Install Ubuntu 22.04

You can either install ubuntu locally or install it on a USB. For this, follow the instructions given on the following links:

* In a PC: https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview
* In a USB: https://www.youtube.com/watch?v=j2RYqahtkNc

### Windows Subsystem Linux (WSL)

It is possible to run the simulation and code for RF labs in a WSL image. For this make sure to have installed WSL () and that you have admin rights in your PC. Then, you can either download Ubuntu 22.04 from the Microsft Store: https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=GB or run the following command in a command prompt:

```
wsl --install Ubuntu-22.04
```

The installer will ask you for a username, make sure you use 'user' as username. It will then ask you for a password, this can be whatever you prefer. After this, you will have a working Ubuntu distro running in Windows. 

## In a Ubuntu 22.04 installation (WSL or Local Installation)

Open a linux terminal and issue the following commands:

*NOTE: if you are using WSL, you will get a terminal ready after the installation ends.*

```
sudo apt update
sudo apt upgrade -y
```

(you can ignore this if you are not using ssh)
```
ssh-keygen -t ed25519 -C "gerardo.aragoncamarasa@glasgow.ac.uk"
cat ~/.ssh/id_ed25519.pub
```

### ROS2 Install
You now can install ROS2 (Humble) by following the instructions in this link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

After installing ROS, run the following commands, each at a time:

```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
echo "export export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
````

### Simulation and RF packages
The next step is to get and build the simulator and related ROS packages for RF. So, run (one line at a time):

```
sudo apt update
sudo apt install ros-humble-libfranka -y
```
```
sudo apt install -y ros-humble-ament-cmake ros-humble-ament-cmake-clang-format ros-humble-angles ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-ros2-control-test-assets ros-humble-controller-manager ros-humble-control-msgs ros-humble-control-toolbox ros-humble-generate-parameter-library ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-moveit ros-humble-pinocchio ros-humble-realtime-tools ros-humble-xacro ros-humble-hardware-interface ros-humble-ros-gz python3-colcon-common-extensions ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```
```
mkdir -p ~/franka_ros2_ws/src
```
```
source ~/.bashrc
```
```
cd ~/franka_ros2_ws/src
```
```
git clone https://github.com/gerac83/rf_simulator.git
```
```
cd ~/franka_ros2_ws
```
```
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```
```
source install/setup.sh
```

**NOTE: If you have a local Ubuntu installation, make sure to replace "user" below (in .../home/user/franka_ros2_ws/...) with your username!**

```
echo "export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/home/user/franka_ros2_ws/src/rf_simulator/" >> ~/.bashrc
```
```
cd ~/franka_ros2_ws/src
```


Only run the following command if you are running WSL with a GPU. For other installation types, this is not needed, just make sure to have your drivers up-to-date.

```
echo "export LIBGL_ALWAYS_SOFTWARE=1"
```

The following command assumes that you have VS Code installed in your Windows environment or in your Ubuntu base installation, if not make sure to download from: https://code.visualstudio.com/

```
code .
```

### Running the Simulation and Motion Planning

Open two terminals (same for VS Code) and issue the following commands in both terminals:
```
cd ~/franka_ros2_ws
source install/setup.sh
```

Everything is now set up so you should be able to run the following commands, start one in each terminal:
1. `ros2 launch franka_gazebo_bringup gazebo_moveit.launch.py`
2. `ros2 launch franka_gazebo_bringup moveit_sim.launch.py`

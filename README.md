# Robotics Foundations Lab Materials - 2025

Welcome to Robotics Foundations (H)! This README provides setup instructions for running the labs, either on a personal computer or using The ConstructSim platform.

## Environment Setup

For running Robotics Foundations Labs, you have two options:

1. Run it on your laptop (this guide), or
2. Use The Construct Sim (free tier) anywhere on the University's lab machines or in your laptop.

### Option 1

The minimum requirements to run the labs in your PC are:

- At least 4 cores, not logical processors (e.g. an Intel Core 6700 has 4 cores with 8 logical processors)
- 8Gb in RAM

Unfortunately, we will not support macOS as virtualisation is close to impossible. The solution is to either use [Boot Camp assistant](https://support.apple.com/en-gb/guide/bootcamp-assistant/bcmp173b3bf2/6.1/mac/15.0), install Ubuntu locally or in a USB (see below), or choose option 2. We will release each lab materials and handout via the course Moodle page.

### Option 2

Just head over [https://app.theconstruct.ai/login] and create an account. Then log in and in the dashboard, type in the search bar *RFLabEnvSetup* to access the `rosject` (`rosject` is what the ConstructSim uses to define projects using ROS). According to the RF schedule, we will release the lab handouts each week in the ConstructSim, you will be able to find them using the search bar by typing:

- *RFLab1* for the first lab in week 2.
- *RFLab2* for the second lab in week 3.
- etc. until lab 4.

**WARNING:** The ConstructSim free tier gives you 8 hours of usage everyday and resets at mighnight. Be careful to **NOT** leave your session open as the clock will keep running and you will run out of time.

**NOTE:** We strongly encourage you to try to setup a local environment as the ConstructSim is slow at times, similar to Google Colab. Moreover, the instructions below show you how to create your own ROS environment, while you will have a ready-to-use environment in the ConstructSim.

After you have found the *RFLabEnvSetup*, open it and familiarise yourself with the platform. You will see that there's a welcome jupyter notebook openned and the taskbar at the bottom has different utilities which you will use such as an integrated instace of VS Code-like IDE, terminals, etc. After this, go to the [Running the Simulation and Motion Planning](#running-the-simulation-and-motion-planning) section.

## Installing Ubuntu 22.04 locally or on a USB

You can either install ubuntu locally or install it on a USB. For this, follow the instructions given on the following links:

- In a PC: [https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- In a USB: [https://www.youtube.com/watch?v=j2RYqahtkNc](https://www.youtube.com/watch?v=j2RYqahtkNc)
- For macOS users: [https://linuxsimply.com/linux-basics/os-installation/dual-boot/ubuntu-on-mac/#:~:text=11%20Steps%20for%20Dual%20Boot%20Installation%20of%20Ubuntu,...%208%208.%20Installation%20Type%20...%20More%20items]

## Windows Subsystem Linux (WSL)

It is possible to run the simulation and code for RF labs in a WSL image on Windows. For this make sure to have installed WSL [https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command](https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command) and that you have admin rights in your PC. Then, you can either download Ubuntu 22.04 from the Microsft Store: [https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=GB](https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=GB) or run the following command in a command prompt:

```bash
wsl --install Ubuntu-22.04
```

The installer will ask you for a username and password, feel free to choose whatever you prefer. After this, you will have a working Ubuntu distro running in Windows.

If you get an error while running WSL, it may mean that you do not have virtualization enable. If this is the case, just ask us during your lab hour so we can help you!

## In a Ubuntu 22.04 installation (WSL, Local or USB Installation)

Open a linux terminal and issue the following commands:

*NOTE: if you are using WSL, you will get a terminal ready after the installation ends.*

```bash
sudo apt update
sudo apt upgrade -y
```

### VS Code Installation in a local Ubuntu installation

To instal VSCode, go to [https://code.visualstudio.com/download](https://code.visualstudio.com/download) and download the `.deb` file for "Debian, Ubuntu". After you have finished downloading the file, open a terminal and type the following:

```bash
cd ~/Downloads
sudo apt install ./<file>.deb
```

You can test whether the installation went OK by typing in the terminal `code`.

## VS Code Installation using WSL

For this, you need to download VS Code for windows which you can find at [https://code.visualstudio.com/download](https://code.visualstudio.com/download). When downloaded, double-click on the file and follow the instructions. Use the default settings.

Now, in your WSL command prompt (or PowerShell) where you the Ubuntu prompt, type `code .` and you will get the Windows version of VS Code connecting to the WSL automatically!

## ROS2 Installation

You now can install ROS2 (Humble) by following the instructions in this link: [CLICK HERE](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

After installing ROS, run the following commands, each at a time:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
echo "export export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

### Simulation and RF packages

The next step is to get and build the simulator and related ROS packages for RF. So, run (one line at a time):

```bash
sudo apt update
sudo apt install ros-humble-libfranka -y
```

```bash
sudo apt install -y ros-humble-ament-cmake ros-humble-ament-cmake-clang-format ros-humble-angles ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-ros2-control-test-assets ros-humble-controller-manager ros-humble-control-msgs ros-humble-control-toolbox ros-humble-generate-parameter-library ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-moveit ros-humble-pinocchio ros-humble-realtime-tools ros-humble-xacro ros-humble-hardware-interface ros-humble-ros-gz python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp python3-ipykernel python3-jupyter-client
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

The following command assumes that you have VS Code installed in your Windows environment or in your Ubuntu base installation, if not make sure to follow the instructions given above.

```bash
code .
```

## Running the Simulation and Motion Planning

Open two terminals (same for VS Code) and issue the following commands in both terminals:

```bash
cd ~/franka_ros2_ws
source install/setup.sh
```

Everything is now set up so you should be able to run the following commands, start one in each terminal:

1. `ros2 launch franka_gazebo_bringup gazebo_empty.launch.py`
2. `ros2 launch franka_gazebo_bringup moveit_sim.launch.py`

and you should be able to see the simulation (aka Gazebo) and a robot visualisation (RViz). Explore both and see if you can make the robot move! Although, that will be the aim of the first lab.

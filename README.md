# Robotics Foundations Lab Materials - 2026

Welcome to Robotics Foundations (H)! This README provides setup instructions for running the labs, either on a personal computer or using The ConstructSim platform.

<!-- vscode-markdown-toc -->
* [1. Environment Setup](#EnvironmentSetup)
* [2. Installing Ubuntu 22.04 locally or on a USB](#InstallingUbuntu22.04locallyoronaUSB)
* [3. Windows Subsystem Linux (WSL)](#WindowsSubsystemLinuxWSL)
* [4. In a Ubuntu 22.04 installation (WSL, Local or USB Installation)](#InaUbuntu22.04installationWSLLocalorUSBInstallation)
	* [4.1. VS Code Installation in a local Ubuntu installation](#VSCodeInstallationinalocalUbuntuinstallation)
	* [4.2. VS Code Installation using WSL](#VSCodeInstallationusingWSL)
* [5. ROS2 Installation](#ROS2Installation)
* [6. RF Environment Setup](#RFEnvironmentSetup)
* [7. Running the Simulation and Motion Planning](#RunningtheSimulationandMotionPlanning)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->

##  1. <a name='EnvironmentSetup'></a>Environment Setup

For running Robotics Foundations Labs, you have two options:

1. Run it on your laptop (this guide), or
2. Use The Construct Sim (free tier) anywhere on the University's lab machines or in your laptop.

**Option 1:** The minimum requirements to run the labs in your PC are:

- At least 4 cores, not logical processors (e.g. an Intel Core 6700 has 4 cores with 8 logical processors)
- 8Gb in RAM

Unfortunately, we will not support macOS as virtualisation is close to impossible. The solution is to either use [Boot Camp assistant](https://support.apple.com/en-gb/guide/bootcamp-assistant/welcome/mac), install Ubuntu locally or in a USB (see below), or choose option 2. We will release each lab materials and handout via the course Moodle page.

**Option 2:** Just head over [https://app.theconstruct.ai/login] and create an account. Then log in and in the dashboard, type in the search bar *RFLabEnvSetup* to access the `rosject` (`rosject` is what the ConstructSim uses to define projects using ROS). ALternatively, you can follow this [direct link](https://app.theconstruct.ai/rosjects/search/?q=RFLabEnvSetup) to the search results.


According to the RF schedule, we will release the lab handouts each week in the ConstructSim, you will be able to find them using the search bar by typing (we will also provide direct links via Moodle):

- *RFLab1* for the first lab in week 2.
- *RFLab2* for the second lab in week 3.
- etc. until lab 4.

**WARNING:** The ConstructSim free tier gives you 8 hours of usage everyday and resets at mighnight. Be careful to **NOT** leave your session open as the clock will keep running and you will run out of time.

**NOTE:** We strongly encourage you to try to setup a local environment as the ConstructSim is slow at times, similar to Google Colab. Moreover, the instructions below show you how to create your own ROS environment, while you will have a ready-to-use environment in the ConstructSim.

After you have found the *RFLabEnvSetup*, make sure that you are forking the `rosject` created by `gerac83` user (this should be the second to last from the search results). Then, rename the `rosject` with your studentid and click save. Finally, click "Run" (green button in the top-right corner) and familiarise yourself with the platform. You will see that there's a welcome jupyter notebook openned and the taskbar at the bottom has different utilities which you will use such as an integrated instace of VS Code-like IDE, terminals, etc.

After this, follow the instructions given in the welcome jupyter notebook. These instructions are similar to the [Running the Simulation and Motion Planning](#running-the-simulation-and-motion-planning) section. If you are interested, you are also free to try to setup your environment by following the instructions in the [RF Environment Setup](rf-environment-setup) section.

##  2. <a name='InstallingUbuntu22.04locallyoronaUSB'></a>Installing Ubuntu 22.04 locally or on a USB

You can either install ubuntu locally or install it on a USB. For this, follow the instructions given on the following links:

- In a PC: [https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- In a USB: [https://www.youtube.com/watch?v=j2RYqahtkNc](https://www.youtube.com/watch?v=j2RYqahtkNc)
- For macOS users: [https://linuxsimply.com/linux-basics/os-installation/dual-boot/ubuntu-on-mac/#:~:text=11%20Steps%20for%20Dual%20Boot%20Installation%20of%20Ubuntu,...%208%208.%20Installation%20Type%20...%20More%20items]

##  3. <a name='WindowsSubsystemLinuxWSL'></a>Windows Subsystem Linux (WSL)

It is possible to run the simulation and code for RF labs in a WSL image on Windows. For this make sure to have installed WSL [https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command](https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command) and that you have admin rights in your PC. Then, you can either download Ubuntu 22.04 from the Microsft Store: [https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=GB](https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=GB) or run the following command in a command prompt:

```bash
wsl --install Ubuntu-22.04
```

The installer will ask you for a username and password, feel free to choose whatever you prefer. After this, you will have a working Ubuntu distro running in Windows.

If you get an error while running WSL, it may mean that you do not have virtualization enable. If this is the case, just ask us during your lab hour so we can help you!

##  4. <a name='InaUbuntu22.04installationWSLLocalorUSBInstallation'></a>In a Ubuntu 22.04 installation (WSL, Local or USB Installation)

Open a linux terminal and issue the following commands:

*NOTE: if you are using WSL, you will get a terminal ready after the installation ends.*

```bash
sudo apt update
sudo apt upgrade -y
```

###  4.1. <a name='VSCodeInstallationinalocalUbuntuinstallation'></a>VS Code Installation in a local Ubuntu installation

**NOTE:** If you are using WSL, please skip this section and go to [VS Code Installation using WSL](#vs-code-installation-using-wsl) section.

To install VSCode, go to [https://code.visualstudio.com/download](https://code.visualstudio.com/download) and download the `.deb` file for "Debian, Ubuntu". After you have finished downloading the file, open a terminal and type the following:

```bash
cd ~/Downloads
sudo apt install ./<file>.deb
```

You can test whether the installation went OK by typing in the terminal `code`.

###  4.2. <a name='VSCodeInstallationusingWSL'></a>VS Code Installation using WSL

If you do not have VS Code installed on Windows, you need to download it from [https://code.visualstudio.com/download](https://code.visualstudio.com/download). When downloaded, double-click on the file and follow the instructions. Use the default settings.

Now, in your WSL command prompt (or PowerShell), type `code .` and you will get the Windows version of VS Code connecting to the WSL automatically!

##  5. <a name='ROS2Installation'></a>ROS2 Installation

You now can install ROS2 (Humble). The commands below are similar to those found in [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Run the following commands in the terminal, each line at a time:

###### Add Ubuntu Universe repo and setup sources

```bash
sudo apt install software-properties-common

sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```

###### Install ROS2 packages; we will use the recommended desktop install

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install ros-humble-desktop -y
```

That's it! :)

##  6. <a name='RFEnvironmentSetup'></a>RF Environment Setup

**Note:** These steps are already executed in the ConstructSim. Do not run them if you are using the ConstructSim!

After installing ROS, you can now setup your environment for RF. For this, run the following commands, each at a time:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

The first line adds to `.bashrc` the ROS2 installation PATH such that you can use ROS2 functionalities from the command line. `.bashrc` is sourced (i.e. executed) everytime you open a terminal. The following two lines tells the terminal interpreter (i.e. bash) to colorised ROS 2 logger and restrict network communications within the localhost (i.e. your PC; otherwise you will observe unpredictable behaviour while running ROS in the university).

The next step is to get and build the simulator and related ROS packages for RF. So, run:

```bash
sudo apt update && sudo apt install -y ros-humble-ament-cmake ros-humble-ament-cmake-clang-format ros-humble-angles ros-humble-ros2-controllers ros-humble-ros2-control ros-humble-ros2-control-test-assets ros-humble-controller-manager ros-humble-control-msgs ros-humble-control-toolbox ros-humble-generate-parameter-library ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-moveit ros-humble-pinocchio ros-humble-realtime-tools ros-humble-xacro ros-humble-hardware-interface ros-humble-ros-gz python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp python3-ipykernel python3-jupyter-client ros-humble-libfranka
```

Cretae a ROS2 workspace for the simulation and motion planning ROS packages:

```bash
mkdir -p ~/franka_ros2_ws/src
```

```bash
cd ~/franka_ros2_ws/
```

Now you can clone `rf_simulator` into this workspace and then source the ROS installation:

```bash
git clone https://github.com/gerac83/rf_simulator.git src/rf_simulator
source ~/.bashrc
```

The following command will build and compile RF's simulation workspace:


```bash
find . -type f -exec touch {} + && colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

This command tells the interprater where the newly built ROS packages are such that you can execute them and adds an environmental variable that points to where the 3D models used for the simulation are.

```bash
export USER="$(whoami)"
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "source /home/${USER}/franka_ros2_ws/install/setup.sh" >> ~/.bashrc
echo "export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/home/${USER}/franka_ros2_ws/src/rf_simulator/" >> ~/.bashrc
```

The following command sources your bashrc such that the above is reflected in your current terminal session. This command is run everytime you open a terminal.

```bash
source ~/.bashrc
```

Then, you can now open the code base for the RF simualtion. First, change directory (i.e `cd`) in the terminal to where the code is located:

```bash
cd ~/franka_ros2_ws/src
```

(If you are using the ConstructSim, do not run this command; instead use the code editor provided in the bottom right panel.) The following command assumes that you have VS Code installed in your Windows environment or in your Ubuntu base installation, if not make sure to follow the instructions given above.

```bash
code .
```

##  7. <a name='RunningtheSimulationandMotionPlanning'></a>Running the Simulation and Motion Planning

Open two terminals (same for VS Code) and issue the following command in both terminals (NOTE: `~` is a shortcut to your home directory path):

```bash
cd ~/franka_ros2_ws
```

Everything is now set up so you should be able to run the following commands, start one in each terminal:

1. `ros2 launch franka_gazebo_bringup gazebo_empty.launch.py`
2. `ros2 launch franka_gazebo_bringup moveit_sim.launch.py`

and you should be able to see the simulation (aka Gazebo) and a robot visualisation (RViz). Explore both and see if you can make the robot move! Although, that will be the aim of the first lab.

To close the simulation and the robot visualisation, just press `Ctrl-C` on both terminals where you run the commands above.

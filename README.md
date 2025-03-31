
# Thailand Open Compettion 2025 (Junior Rescue)

It is a robot designed and developed specifically for rescue operations, helping to access areas that are too dangerous or impossible for humans to reach. Equipped with advanced sensors, autonomous navigation, and precise control systems, this type of robot can operate in disaster zones, collapsed buildings, or hazardous environments, ensuring that rescue missions can be carried out efficiently and safely.

## Office Resources
* [IRAP](https://github.com/TanakornKulsri/iRAP_RMRC?tab=readme-ov-file)
* [RoboCupJunior Rescue](https://junior.robocup.org/rescue/)

## SetUp
* [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
* [ROS SetUp (Noetic)](https://wiki.ros.org/noetic/Installation/Ubuntu)
*[Create a workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)

## Create New Workspace

open terminal

```bash
mkdir -p ~/rmrc_ws/src
cd ~/rmrc_ws/
catkin_make
```

```bash
source devel/setup.bash
```
```bash
nano .bashrc
```

Add Commment in .bashrc
```bash
source ~/rmrc_ws/devel/setup.bash
```

in terminal
```bash
gitclone https://github.com/RETOUTHz/ACS-ROBORES?tab=readme-ov-file
```






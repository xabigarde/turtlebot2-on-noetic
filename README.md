# turtlebot2-on-noetic

My installing procedure about turtlebot2 on neotic (ubuntu 20.04 focal).

## 1. Install ROS Noetic

Below is a summary of the original instructions here: http://wiki.ros.org/noetic/Installation/Ubuntu

I tried to install ros-noetic-desktop-full in an Intel NUC i5 running a clean install of Ubuntu 20.04, but I got an ''unmet dependencies'' error:

```bash
~$ sudo apt install ros-noetic-desktop-full
Reading package lists... Done
Building dependency tree       
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 ros-noetic-desktop-full : Depends: ros-noetic-simulators but it is not going to be installed
                           Depends: ros-noetic-urdf-sim-tutorial but it is not going to be installed
```

I really did not need Gazebo etc so I installed ros-noetic-desktop instead:

```bash
sudo apt install ros-noetic-desktop
```

Once installed, remember to source your installation!!! And optionally, source it through your ~/.bashrc file:

```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

Remember to also install the dependencies for building packages:

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
And initialize rosdep:

```
sudo rosdep init
rosdep update
```

## 2. Download & build Kobuki packages from source

NOTE: the following instructions were last tested on 2021/05/04. The git repositories and their default branches may have changed since then. Do as I did, and manually verify each and every ```git clone``` command's repository for updated branches before running it.

```bash
# create workspace in user's home dir
cd ~
mkdir kobuki_ws/src/ -p
cd kobuki_ws/src/

# Clone some source code (from most up-to-date repositories and branches)
git clone --branch melodic https://github.com/yujinrobot/kobuki.git
git clone --branch noetic https://github.com/yujinrobot/kobuki_msgs.git
git clone --branch noetic https://github.com/yujinrobot/kobuki_core.git

# Download yujin_ocs:
git clone https://github.com/yujinrobot/yujin_ocs.git
# and then remove the unnecessary yocs packages (we only need yocs_cmd_vel_mux & yocs_controllers)
mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers .
rm -rf yujin_ocs/

# ecl_*
# NOTE: MANUALLY CHECK FOR UPDATES in each repo
git clone --branch release/0.61-noetic https://github.com/stonier/ecl_tools.git
git clone --branch release/0.61-noetic https://github.com/stonier/ecl_lite.git
git clone --branch release/0.62-noetic https://github.com/stonier/ecl_core.git
git clone --branch release/0.60-noetic https://github.com/stonier/ecl_navigation.git 

# make
cd ../
catkin_make

# source
source ./devel/setup.bash
echo "~/kobuki_ws/devel/setup.bash" >> ~/.bashrc
```

### Tips

+ Only clone the `yujinrobot/kobuki` repo and `catkin_make` it will generate `Could not find a package configuration file provided by "kobuki_msgs"`, so add `yujinrobot/kobuki_msgs`.
+ Then error messages turn to `Could not find a package configuration file provided by "yocs_controllers"`, with referenced from [install_basic.sh](https://github.com/gaunthan/Turtlebot2-On-Melodic/blob/master/install_basic.sh), it needs to clone `yujinrobot/yujin_ocs`.
+ Same, `Could not find a package configuration file provided by "kobuki_dock_drive"`, add `yujinrobot/kobuki_core`.
+ Same, `Could not find a package configuration file provided by "ecl_mobile_robot"`, with referenced from [kobuki_core.rosinstall](https://raw.githubusercontent.com/yujinrobot/kobuki_core/melodic/kobuki_core.rosinstall), add `stonier/ecl_*`.
+ Finally, `source ./devel/setup.bash` and finish.

## 3. Download and build Turtlebot2 packages from source

+ Inspired by [Is it possible to install Turtlebot software on ROS Noetic ??](https://answers.ros.org/question/355435/hi-i-am-using-kubuntu-2004-and-have-installed-ros-noetic-on-it-is-it-possible-to-install-turtlebot-software-on-ros-noetic-i-couldnot-find-any-liks-any/)

> All turtlebot packages are fully stable on ROS Melodic so if you want to have those in a Noetic Distribution you may want to clone all the repositories in you catkin workspace and compile them in you enviroment. If you install all needed dependencies you should not have any problem since, generally speaking, any package stable in Melodic is likely to work properly under Noetic.

```bash
# create turtlebot2 workspace in user's home dir:
cd ~
mkdir turtlebot2_ws/src/ -p
cd turtlebot2_ws/src/

# clone Turtlebot2 sources:
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_simulator
git clone --branch indigo https://github.com/turtlebot/turtlebot_viz.git

# make
cd ..
catkin_make

# source
source ./devel/setup.bash
echo "~/turtlebot2_ws/devel/setup.bash" >> ~/.bashrc
```

### Tips

+ These procedures have been adapted from https://github.com/Aoi-hosizora/turtlebot2-on-noetic
+ which are almost the same as https://github.com/gaunthan/Turtlebot2-On-Melodic.
+ If `kobuki` is not be installed first, it will generate some errors: `Could not find a package configuration file provided by "kobuki_driver"`.
+ `turtlebot/turtlebot_viz` install `rviz` for `turtlebot_rviz_launchers`.

## Work with turtlebot2

### Bring Up

```bash
roslaunch turtlebot_bringup minimal.launch
```

### Control by KeyBoard

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

### Work with rviz

```bash
roslaunch turtlebot_rviz_launchers view_robot.launch
```

### Some problem

1. Kobuki : device does not (yet) available, is the usb connected?.

> Then do the obvious – make sure kobuki is turned on (leds should be active on the kobuki) and the cable is plugged in. If you are sure about both of these, check to see that your system has had the udev rule applied for /dev/kobuki

```bash
ls -n /dev | grep kobuki # nothing
rosrun kobuki_ftdi create_udev_rules
ls -n /dev | grep kobuki # has one
```

2. Other see [turtlebot移动机器人（05）：turtlebot-Kobuki和Kinect](https://robot-ros.com/robot/37686.html)

## References

+ [gaunthan/Turtlebot2-On-Melodic:install_basic.sh](https://github.com/gaunthan/Turtlebot2-On-Melodic/blob/master/install_basic.sh)
+ [yujinrobot/kobuki_core:kobuki_core.rosinstall](https://github.com/yujinrobot/kobuki_core/blob/melodic/kobuki_core.rosinstall)
+ [Is it possible to install Turtlebot software on ROS Noetic ??](https://answers.ros.org/question/355435/hi-i-am-using-kubuntu-2004-and-have-installed-ros-noetic-on-it-is-it-possible-to-install-turtlebot-software-on-ros-noetic-i-couldnot-find-any-liks-any/)
+ [Jetson Xavier: Turtlebot2をROS Melodicで動かす](https://demura.net/robot/athome/15887.html)
+ [turtlebot移动机器人（05）：turtlebot-Kobuki和Kinect](https://robot-ros.com/robot/37686.html)

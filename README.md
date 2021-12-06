#  Gripper controller 

### Description
This is the control interface for the  gripper. 



------



------



### Installation

1. Make sure you have already installed and Configured Your [ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)  Environment

2. Create and build a [catkin workspace](http://wiki.ros.org/catkin/workspaces): (you can skip this step , if you have already created it)

   ```
   $ mkdir -p ~/catkin_ws/src
   $ cd ~/catkin_ws/
   $ catkin_make
   ```

4. Clone the whole directory( `dh_gripper_driver_ros` ) to  your catkin's workspace src folder

5. Compile the code 

   ```
   $ cd ~/catkin_ws/
   $ catkin_make
   ```

6. Add the path including to the `ROS_PACKAGE_PATH` environment variable. Open `~/.bashrc` file and add at the end the following line. 

   ```
   $ source ~/catkin_ws/devel/setup.bash
   ```

------



### Instructions

1. ##### Connect all hardware and Turn on the power

2. ##### Modify launch file and Running controller

   First , Modify the  `dh_gripper.launch` file in according to the product model 

      ```
       //gripperID default 1
       <arg name="GripperID" default="1"/>
       //gripper Model
       <arg name="GripperModel" default="PGE"/>
       //grippper USB Port Name
       <arg name="Connectport" default="/dev/ttyUSB0"/>
       //gripper Baudrate defalut : 115200
       <arg name="Baudrate" default="115200"/>	
      ```

   Now , you can running controller

   ```
   $ roslaunch dh_gripper_driver dh_gripper.launch
   ```

   > If If it runs successfully , you will see the initialization of the gripper,and then auto close and open

3. ##### Disable test run ,and use topic

    Modify the  `dh_gripper.launch` file, and roslaunch `dh_gripper.launch`

   ```
     <arg name="test_run" default="false"/>
   ```

   use topic  to control
   
   ```
   $rostopic pub /gripper/ctrl dh_gripper_msgs/GripperCtrl "initialize: false
   position: 0.0
   force: 100.0
   speed: 100.0" 
   ```

4. ##### Study how to use it 

   you can read the ` dh_gripper_Test.cpp`  in `/dh_hand_driver/src` folder to study

5. ##### Enjoy it



### gazebo7
   $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

   ```
   $ catkin_make



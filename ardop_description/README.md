# ARDOP 2.0 - URDF Description

## Setup
 
1. Install Gazebo  
2. Install Gazebo ROS Plugins  
3. Install RViz  
4. Install Joint State Publisher  

## Usage

#### Build the Package
Copy the package to ```~/catkin_ws/src/``` folder.

Then just build the package:

```
cd ~/catkin_ws/
catkin_make
source devel/setup.sh
```

#### Run the Package

To visualize in RViz:
  
```roslaunch ardop_description_01 display.launch```

To visualize in Gazebo:
  
```roslaunch ardop_description_01 gazebo.launch```

## Notes

#### Tutorials
1. [Catkin Tutorials](http://wiki.ros.org/catkin/Tutorials)
2. [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
3. [Gazebo URDF Tutorial](http://gazebosim.org/tutorials/?tut=ros_urdf)
4. [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)




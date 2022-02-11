A1_visualizer


This visualizer visulize your A1 robot in rviz



Dependency:

Other than ROS itself (only tested in ROS melodic)

1. sudo apt install libpcl-dev
2. tranform graph  https://github.com/jstnhuang/transform_graph
3. sudo apt-get install ros-melodic-orocos-kdl
4. Unitree ROS     https://github.com/paulyang1990/unitree_ros
   
   Follow the README carefully, and remember the unitree_legged_sdk version must be v3.2. Refer to A1-Docker [dockerfile](https://github.com/paulyang1990/A1-Docker/blob/main/docker/Dockerfile) for useful commands



The visualizer will search for unitree ros package installed on your machine, then read urdf from unitree ros package. 

Please make sure you can roscd unitree_legged_real before running this package.



The visualization result looks like this:
![visualize_image](doc/images/a1_visualizer.png)

(The camera frame and points in the scene comes from VINS-Fusion. They are not part of this package)
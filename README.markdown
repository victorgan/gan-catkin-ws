# Workspace
Here's my workspace for locally-installed ROS packages.

# What's in here
beginner_tutorials http://wiki.ros.org/catkin/Tutorials/using_a_workspace
Learning a workspace

my_pcl_tutorial http://wiki.ros.org/pcl/Tutorials (click 'Hydro')
Using PCL with ROS


# To Use
source /opt/ros/indigo/setup.bash # use installation space this bash instance
cd gan-catkin-ws
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH # ensure gan-catkin-ws is listed

Then,
For beginner_tutorials:
> roslaunch beginner_tutorials turtle mimic.launch

# More Information
http://wiki.ros.org/catkin/workspaces
http://wiki.ros.org/catkin/Tutorials/using_a_workspace

# RGBDSLAM
RGBDSlam requires openCV's nonfree libraries.

sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update 
sudo apt-get install libopencv-nonfree-dev

# Tree
gan-catkin-ws/         -- WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file, symbolic link to catkin's boiler-plate 'toplevel' CMakeLists.txt file
    package_1/
      CMakeLists.txt
      package.xml
      ...
    
    beginner_tutorials/   -- from: http://wiki.ros.org/catkin/Tutorials/using_a_workspace 

  build/                  -- BUILD SPACE
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)


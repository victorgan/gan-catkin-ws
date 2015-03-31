# Workspace
Here's my workspace for locally-installed ROS packages.

# To Use
source /opt/ros/indigo/setup.bash # use installation space this bash instance
cd gan-catkin-ws
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH # ensure gan-catkin-ws is listed

# More Information
http://wiki.ros.org/catkin/workspaces
http://wiki.ros.org/catkin/Tutorials/using_a_workspace

# Tree
gan-catkin-ws/         -- WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file, symbolic link to catkin's boiler-plate 'toplevel' CMakeLists.txt file
    package_1/
      CMakeLists.txt
      package.xml
      ...
    package_n/
      CMakeLists.txt
      package.xml
      ...
  build/                  -- BUILD SPACE
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)

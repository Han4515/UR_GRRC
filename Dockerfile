# syntax=docker/dockerfile:1
FROM osrf/ros:melodic-desktop-full

# Create catkin workspace
# Install UR Robot Driver

# UR A
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/melodic/setup.bash &&\
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws &&\
    git clone -b staging https://github.com/Han4515/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver &&\
    git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot &&\    
    sudo apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# UR B
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/melodic/setup.bash &&\
    mkdir -p ~/catkin_ws2/src && cd ~/catkin_ws2 &&\
    git clone -b staging https://github.com/Han4515/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver &&\
    git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot &&\    
    sudo apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    cd ~/catkin_ws2 &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Packages for Installing Python Packages
RUN /bin/bash -c "apt-get update -y &&\
    apt-get install -y python-dev &&\
    apt-get install -y build-essential &&\
    apt-get install -y libssl-dev &&\
    apt-get install -y libffi-dev &&\
    apt-get install -y libxml2-dev &&\
    apt-get install -y libxslt1-dev &&\
    apt-get install -y zlib1g-dev &&\
    apt-get install -y python-pip"

# Install Pynput Package for Keyboard Input Control
RUN python -m pip install pynput

# Install Custom Package(ur_ros_joint_control)

# UR A
RUN /bin/bash -c "cd ~/catkin_ws &&\
    git clone https://github.com/Han4515/ur_ros_joint_control.git src/ur_ros_joint_control &&\
    source /opt/ros/melodic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"

# UR B
RUN /bin/bash -c "cd ~/catkin_ws2 &&\
    git clone https://github.com/Han4515/ur_ros_joint_control.git src/ur_ros_joint_control &&\
    source /opt/ros/melodic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Cartesian Controller Packages

# UR A
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs.git src/Universal_Robots_ROS_cartesian_control_msgs &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian.git src/Universal_Robots_ROS_controllers_cartesian &&\
    sudo apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    catkin_make &&\
    source devel/setup.bash"

# UR B
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws2 &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs.git src/Universal_Robots_ROS_cartesian_control_msgs &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian.git src/Universal_Robots_ROS_controllers_cartesian &&\
    sudo apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    catkin_make &&\
    source devel/setup.bash"

# Install scipy
RUN python -m pip install scipy
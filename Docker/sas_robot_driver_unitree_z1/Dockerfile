#FROM ros:jazzy-ros-core-noble
FROM ubuntu:noble

SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

# Install dependencies
RUN apt-get update && apt-get install -y net-tools iputils-ping \
git nss-updatedb locate curl libpci3 vim nano terminator \
cmake extra-cmake-modules g++ libeigen3-dev fakeroot python3-pip \
libzmq3-dev libboost-all-dev python3-pybind11 software-properties-common 

# Install ROS2 as instructed in https://ros2-tutorial.readthedocs.io/en/latest/installation.html
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt upgrade -y
RUN apt-get update && apt-get install -y ros-jazzy-desktop ros-dev-tools
# ros-jazzy-desktop



# Install DQ Robotics
RUN add-apt-repository ppa:dqrobotics-dev/development 
RUN apt-get update && apt-get install libdqrobotics libdqrobotics-interface-json11
RUN python3 -m pip install dqrobotics --pre --break-system-packages
RUN apt-get update && apt-get install -y libdqrobotics-interface-coppeliasim libdqrobotics-interface-coppeliasim-zmq

# Install qpOASES
RUN mkdir -p ~/utils
RUN cd ~/utils && git clone https://github.com/coin-or/qpOASES.git
RUN cd ~/utils/qpOASES && sed -i -e 's/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" OFF)/option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" ON)/g' CMakeLists.txt
RUN cd ~/utils/qpOASES/ && mkdir -p build
RUN cd ~/utils/qpOASES/build  && cmake .. && make && make install

# Install DQ Robotics interface with qpOASES
RUN apt-get update && apt-get install -y libdqrobotics-interface-qpoases


# Suggested instructions to clone SAS
RUN mkdir -p ~/ros2_ws/src
RUN cd ~/ros2_ws/src && git clone --recurse-submodules -b jazzy https://github.com/SmartArmStack/smart_arm_stack_ROS2.git sas
RUN cd ~/ros2_ws/src/sas/ && rm -r sas_robot_driver_denso 
RUN cd ~/ros2_ws/src/sas/ && rm -r sas_robot_driver_kuka
RUN cd ~/ros2_ws/src/sas/ && rm -r sas_robot_driver_ur

# Clone the sas driver
RUN cd ~/ros2_ws/src && git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_z1 --recursive

RUN echo "# Update the environment variable LD_LIBRARY_PATH to include ~/opt/lib, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/" >> ~/.bashrc

RUN echo "# Update the environment variable LIBRARY_PATH to include ~/opt/lib, as instructed in https://ros2-tutorial.readthedocs.io" >> ~/.bashrc
RUN echo "export LIBRARY_PATH=$LIBRARY_PATH:~/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/" >> ~/.bashrc


# ALIASES
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "# Source the ROS2 overlay, as instructed in https://github.com/SmartArmStack/smart_arm_stack_ROS2" >> ~/.bashrc
RUN echo "alias cdros2='cd ~/ros2_ws/' " >> ~/.bashrc 
RUN echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> ~/.bashrc 
# ROS DOMAIN SETUP
RUN echo "export ROS_DOMAIN_ID=77" >> ~/.bashrc 
RUN echo "alias start_ROS2_drivers='cdros2 && source install/setup.bash && ros2 launch sas_robot_driver_unitree_z1 real_robot_launch.xml ' " >> ~/.bashrc 

RUN source ~/.bashrc

# To prevent compilation erros with Pybind11
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
RUN ldconfig

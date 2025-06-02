FROM ros:jazzy-ros-core-noble

SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"



RUN apt-get update && apt-get install -y ros-dev-tools ros-jazzy-desktop 
# ros-jazzy-desktop

# Install dependencies
RUN apt-get update && apt-get install -y net-tools iputils-ping \
git nss-updatedb locate curl libpci3 vim nano terminator \
cmake extra-cmake-modules g++ libeigen3-dev fakeroot python3-pip \
libzmq3-dev libboost-all-dev python3-pybind11 software-properties-common 

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




# ALIASES
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "# Source the ROS2 overlay, as instructed in https://github.com/SmartArmStack/smart_arm_stack_ROS2" >> ~/.bashrc
RUN echo "alias cdros2='cd ~/ros2_ws/' " >> ~/.bashrc 
RUN echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> ~/.bashrc 
# ROS DOMAIN SETUP
RUN echo "export ROS_DOMAIN_ID=77" >> ~/.bashrc 
RUN echo "alias start_ROS2_drivers='cdros2 && source install/setup.bash && ros2 launch sas_robot_driver_unitree_b1 real_b1z1_robot_launch.xml ' " >> ~/.bashrc 

RUN source ~/.bashrc

# To prevent compilation erros with Pybind11
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
RUN ldconfig

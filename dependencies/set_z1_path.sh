echo "# Update the environment variable LD_LIBRARY_PATH as instructed in https://ros2-tutorial.readthedocs.io" >> /etc/bash_env
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/" >> /etc/bash_env

echo "# Update the environment variable LIBRARY_PATH as instructed in https://ros2-tutorial.readthedocs.io" >> /etc/bash_env
echo "export LIBRARY_PATH=$LIBRARY_PATH:~/ros2_ws/src/sas_robot_driver_unitree_z1/src/z1_sdk/lib/" >> /etc/bash_env

echo "alias launch_ROS2_Z1_drivers='cdros2 && source install/setup.bash && ros2 launch sas_robot_driver_unitree_z1 real_z1_robot_launch.py ' " >> /etc/bash_env

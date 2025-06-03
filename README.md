![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)


# sas_robot_driver_unitree_z1

### Docker Instructions

#### Prerequisites:
- Docker installed with sudo permisions.
- [Prepare the Unitree Z1 arm](link here).

1. Clone this repository
```shell
cd ~/Downloads
git clone https://github.com/Adorno-Lab/sas_robot_driver_unitree_z1 --recursive
cd sas_robot_driver_unitree_z1
```
2. Build the docker image
```shell
sh build_sas_rd_unitree_z1_docker.sh 
```
3. Start the docker container
```shell
sh start_sas_rd_unitree_z1_docker.sh  
```
4. Start the driver
```shell
start_ROS_drivers
```

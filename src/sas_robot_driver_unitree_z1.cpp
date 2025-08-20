#include "sas_robot_driver_unitree_z1/sas_robot_driver_unitree_z1.hpp"

#include "DriverUnitreeZ1.hpp"
#include <iostream>
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>


namespace sas
{

class RobotDriverUnitreeZ1::Impl
{

public:
    std::shared_ptr<DriverUnitreeZ1> unitree_z1_driver_;
    Impl()
    {

    };



};

RobotDriverUnitreeZ1::RobotDriverUnitreeZ1(std::shared_ptr<Node> &node,
                                           const RobotDriverUnitreeZ1Configuration &configuration,
                                           std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    configuration_(configuration),
    node_{node},
    topic_prefix_{configuration.robot_name}
{
    impl_ = std::make_unique<RobotDriverUnitreeZ1::Impl>();

    // Extract the driver mode from configuration.mode
    DriverUnitreeZ1::MODE mode;
    if (configuration.mode == "PositionControl")
        mode = DriverUnitreeZ1::MODE::PositionControl;
    else
        throw std::runtime_error("The PositionControl mode is the only one supported!");


    impl_->unitree_z1_driver_ = std::make_shared<DriverUnitreeZ1>(break_loops,
                                                                  mode,
                                                                  configuration.gripper_attached,
                                                                  configuration.verbosity);
    joint_limits_ = configuration.joint_limits;

    subscriber_target_joint_raw_commands_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_prefix_ + "/set/target_joint_raw_commands",
        1,
        std::bind(&RobotDriverUnitreeZ1::_callback_target_joint_raw_commands, this, std::placeholders::_1)
        );
}

VectorXd RobotDriverUnitreeZ1::get_joint_positions()
{
    return impl_->unitree_z1_driver_->get_joint_positions_with_gripper();
}

void RobotDriverUnitreeZ1::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    impl_->unitree_z1_driver_->set_target_joint_positions_with_gripper(desired_joint_positions_rad);
}

VectorXd RobotDriverUnitreeZ1::get_joint_velocities()
{
    return impl_->unitree_z1_driver_->get_joint_velocities_with_gripper();
}

void RobotDriverUnitreeZ1::connect()
{
    impl_->unitree_z1_driver_->connect();
}

void RobotDriverUnitreeZ1::disconnect()
{
    impl_->unitree_z1_driver_->disconnect();
}

void RobotDriverUnitreeZ1::initialize()
{
    impl_->unitree_z1_driver_->move_robot_to_forward_position_when_initialized();
    impl_->unitree_z1_driver_->initialize();
}

void RobotDriverUnitreeZ1::deinitialize()
{
    impl_->unitree_z1_driver_->deinitialize();
}


void RobotDriverUnitreeZ1::_callback_target_joint_raw_commands(const std_msgs::msg::Float64MultiArray &msg)
{
    target_raw_commands_  = std_vector_double_to_vectorxd(msg.data);
    new_target_velocities_available_ = true;
}

RobotDriverUnitreeZ1::~RobotDriverUnitreeZ1()
{

}


}

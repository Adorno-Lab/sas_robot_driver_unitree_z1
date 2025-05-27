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

RobotDriverUnitreeZ1::RobotDriverUnitreeZ1(const RobotDriverUnitreeZ1Configuration &configuration, std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
{
    impl_ = std::make_unique<RobotDriverUnitreeZ1::Impl>();

    // I need to use the parameters of the configuration structure!
    impl_->unitree_z1_driver_ = std::make_shared<DriverUnitreeZ1>(break_loops, DriverUnitreeZ1::MODE::PositionControl, true, true);
    joint_limits_ = configuration.joint_limits;
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


RobotDriverUnitreeZ1::~RobotDriverUnitreeZ1()
{

}


}

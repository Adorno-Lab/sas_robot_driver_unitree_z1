
#include <rclcpp/rclcpp.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_robot_driver_unitree_z1/sas_robot_driver_unitree_z1.hpp>
#include <dqrobotics/utils/DQ_Math.h>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{

    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);

    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_unitree_z1");


    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverUnitreeZ1Configuration configuration;

        sas::get_ros_parameter(node,"gripper_attached",configuration.gripper_attached);
        sas::get_ros_parameter(node,"mode",configuration.mode);
        sas::get_ros_parameter(node,"verbosity",configuration.verbosity);
        sas::get_ros_parameter(node,"move_robot_to_initial_configuration",configuration.move_to_initial_configuration);
        sas::get_ros_parameter(node, "open_loop_joint_control_gain", configuration.open_loop_joint_control_gain);

        std::vector<double> initial_configuration;
        sas::get_ros_parameter(node,"initial_configuration",initial_configuration);
        configuration.initial_configuration = deg2rad(sas::std_vector_double_to_vectorxd(initial_configuration));

        std::vector<double> joint_limits_min;
        std::vector<double> joint_limits_max;
        sas::get_ros_parameter(node,"joint_limits_min",joint_limits_min);
        sas::get_ros_parameter(node,"joint_limits_max",joint_limits_max);
        configuration.joint_limits = {deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_min)),
                                      deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_max))};


        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverUnitreeZ1.");
        auto robot_driver_unitree_z1 = std::make_shared<sas::RobotDriverUnitreeZ1>(configuration,
                                                                                   &kill_this_process);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_unitree_z1,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    return 0;
}

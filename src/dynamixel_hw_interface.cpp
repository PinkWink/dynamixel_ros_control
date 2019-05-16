#include <dynamixel_ros_control/dynamixel_hw_interface.h>

DynamixelHWInterface::DynamixelHWInterface() {

}

DynamixelHWInterface::~DynamixelHWInterface() {

}

bool DynamixelHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
    return true;
}

void DynamixelHWInterface::read(const ros::Time& time, const ros::Duration& period)
{
    ROS_INFO("read");
}

void DynamixelHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
    ROS_INFO("write");
}

bool DynamixelHWInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    return true;
}

void DynamixelHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
}
#ifndef DYNAMIXEL_HW_INTERFACE_H_
#define DYNAMIXEL_HW_INTERFACE_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

class DynamixelHWInterface: public hardware_interface::RobotHW
{
    public:
        DynamixelHWInterface();
        ~DynamixelHWInterface();

    public:
        virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
        virtual void read(const ros::Time& time, const ros::Duration& period);
        virtual void write(const ros::Time& time, const ros::Duration& period);
        virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
        virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    private:
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;
        dynamixel::GroupBulkRead *groupBulkRead;
};

#endif //DYNAMIXEL_HW_INTERFACE_H_
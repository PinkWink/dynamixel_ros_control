#ifndef DYNAMIXEL_HW_INTERFACE_H_
#define DYNAMIXEL_HW_INTERFACE_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_ros_control/dynamixel_info.h"
#include "dynamixel_ros_control/dynamixel_motor.h"
#include <ros/ros.h>

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

    public:
        bool is_ready();

    private:
        dynamixel::PortHandler *portHandler_;
        dynamixel::PacketHandler *packetHandler_;
        dynamixel::GroupBulkRead *groupBulkRead_;
        dynamixel::GroupBulkWrite *groupBulkWrite_;

        //std::map<int, DynamixelMotor*> mapDynamixelMotors_;
        std::vector<DynamixelMotor*> dynamixel_motors_;

        hardware_interface::JointStateInterface jnt_state_interface_;
        hardware_interface::PositionJointInterface jnt_pos_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        hardware_interface::EffortJointInterface jnt_eff_interface_;

        std::vector<double> joint_cmd_;
        std::vector<double> joint_pos_;
        std::vector<double> joint_vel_;
        std::vector<double> joint_eff_;
};

#endif //DYNAMIXEL_HW_INTERFACE_H_
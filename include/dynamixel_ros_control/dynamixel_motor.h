#ifndef DYNAMIXEL_MOTOR_H_
#define DYNAMIXEL_MOTOR_H_

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_ros_control/dynamixel_info.h"
#include "dynamixel_ros_control/dynamixel_control_table.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

class DynamixelMotor
{
public:
    DynamixelMotor(dynamixel::PortHandler *port, dynamixel::PacketHandler* packet, dynamixel::GroupBulkRead *read, dynamixel::GroupBulkWrite *write);
    ~DynamixelMotor();

public:
    bool init(int model_number, std::string name);
    bool update();
    bool write(double cmd);
    void stop();

    std::string get_joint_name();
    void get_current_value(double &pos, double &vel, double &effort);

private:
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupBulkRead *groupBulkRead_;
    dynamixel::GroupBulkWrite *groupBulkWrite_;

    int motor_model_num_;
    DynamixelSeries dynamixel_series_;
    std::string motor_name_;
    std::string joint_name_;
    double joint_gear_ratio_;
    double joint_inverse_;
    int motor_id_;
    int operating_mode_;
    double profile_acceleration_;

    double joint_pos_;
    double joint_vel_;
    double joint_eff_;
};


#endif //DYNAMIXEL_MOTOR_H_
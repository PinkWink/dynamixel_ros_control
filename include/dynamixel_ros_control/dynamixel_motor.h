#ifndef DYNAMIXEL_MOTOR_H_
#define DYNAMIXEL_MOTOR_H_

#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_ros_control/dynamixel_info.h"
#include "dynamixel_ros_control/dynamixel_control_table.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "dynamixel_ros_control/HomingAction.h"
#include <actionlib/server/simple_action_server.h>


class DynamixelMotor
{
public:
    DynamixelMotor(dynamixel::PortHandler *port, dynamixel::PacketHandler* packet, dynamixel::GroupBulkRead *read, dynamixel::GroupBulkWrite *write);
    ~DynamixelMotor();

public:
    bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh, int model_number, std::string name);
    bool update();
    bool write(double cmd);
    void stop();

    bool is_ready();
    std::string get_joint_name();
    void get_current_value(double &pos, double &vel, double &effort);

private:
    void execute_homing(const dynamixel_ros_control::HomingGoalConstPtr &goal);
    bool init_and_ready(bool skip_read_register);

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
    double profile_velocity_;
    int origin_offset_;
    int velocity_p_gain_;
    int velocity_i_gain_;

    boost::shared_ptr<actionlib::SimpleActionServer<dynamixel_ros_control::HomingAction>> homing_as_;
    bool need_homing_;
    bool is_ready_;
    bool is_homing_;
    int homing_mode_;
    double homing_direction_;
    double homing_max_speed_;
    double homing_current_limit_;
    double homing_offset_;

    bool is_gripper_;
    double gripper_gap_size_;
    double gripper_current_limit_;
    double gripper_stroke_;

    double joint_pos_;
    double joint_vel_;
    double joint_eff_;
};


#endif //DYNAMIXEL_MOTOR_H_
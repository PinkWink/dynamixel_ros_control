#include "dynamixel_ros_control/dynamixel_control_table.h"
#include "dynamixel_ros_control/dynamixel_motor.h"


DynamixelMotor::DynamixelMotor(dynamixel::PortHandler *port, dynamixel::PacketHandler* packet, dynamixel::GroupBulkRead *read, dynamixel::GroupBulkWrite *write)
{
    portHandler_ = port;
    packetHandler_ = packet;
    groupBulkRead_ = read;
    groupBulkWrite_ = write;

    joint_gear_ratio_ = 1.0;
    joint_inverse_ = 1.0;
    joint_pos_ = 0.0;
    joint_vel_ = 0.0;
    joint_eff_ = 0.0;

    is_ready_ = true;
    is_homing_ = false;
}

DynamixelMotor::~DynamixelMotor()
{
}

bool DynamixelMotor::init(int model_number, std::string name)
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    motor_name_ = name;
    motor_model_num_ = model_number;
    dynamixel_series_ = DynamixelModel[motor_model_num_];

    pnh.getParam(name + "/id", motor_id_);
    pnh.getParam(name + "/operating_mode", operating_mode_);
    pnh.getParam(name + "/joint_name", joint_name_);
    pnh.getParam(name + "/gear_ratio", joint_gear_ratio_);
    pnh.getParam(name + "/inverse", joint_inverse_);
    pnh.getParam(name + "/profile_acceleration", profile_acceleration_);

    pnh.param<bool>(name + "/homing", need_homing_, false);
    if(need_homing_)
    {
        pnh.getParam(name + "/homing_mode", homing_mode_);
        pnh.getParam(name + "/homing_direction", homing_direction_);

        ROS_INFO("[%s] motor need homing.", motor_name_.c_str());
        is_homing_ = false;
        homing_as_ = boost::make_shared<actionlib::SimpleActionServer<dynamixel_ros_control::HomingAction>>(nh, name + "/homing", boost::bind(&DynamixelMotor::execute_homing, this, _1), false);
		homing_as_->start();

        is_ready_ = false;
    }

    return init_and_ready(false);
}

bool DynamixelMotor::is_ready()
{
    return is_ready_;
}

bool DynamixelMotor::init_and_ready(bool skip_read_register)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // reboot first
    if(packetHandler_->reboot(portHandler_, motor_id_, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to reboot motor [%s]", motor_name_.c_str());
        return false;
    }
    ros::Duration(0.3).sleep();

    // set operating mode
    ROS_INFO("[%s] set operating mode to [%d]...", motor_name_.c_str(), operating_mode_);
    if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::OPERATING_MODE], (uint8_t)operating_mode_, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set operating mode [%d] on [%s].", motor_id_, motor_name_.c_str());
        return false;
    }

    // torque enable
    if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_, DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::TORQUE_ENABLE], 1, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set torque enable [%d] on [%s].", motor_id_, motor_name_.c_str());
        return false;
    }

    // set profile_accelration
    uint32_t target_acceleration = profile_acceleration_ / (0.01 * M_PI / 60.0) * joint_gear_ratio_;
    if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_, DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PROFILE_ACCELERATION], target_acceleration, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set torque enable [%d] on [%s].", motor_id_, motor_name_.c_str());
        return false;
    }

    // register for status reading.
    if(!skip_read_register)
    {
        if(!groupBulkRead_->addParam(motor_id_,
                DynamixelReadStartAddress[dynamixel_series_], DynamixelReadLength[dynamixel_series_]))
        {
            ROS_ERROR("Failed to addParam position");
        }
    }
}

bool DynamixelMotor::update()
{
    //ROS_INFO("[%s] update", motor_name_.c_str());

    if(groupBulkRead_->isAvailable(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_POSITION], 4))
    {
        int32_t position = groupBulkRead_->getData(motor_id_,
                DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_POSITION], 4);

        switch(motor_model_num_)
        {
            case PRO_PLUS_H54P_200_S500_R:
            case PRO_PLUS_H54P_100_S500_R:
                joint_pos_ = position * M_PI / 501923.0 / joint_gear_ratio_ * joint_inverse_;
                break;
            case PRO_PLUS_H42P_020_S300_R:
                joint_pos_ = position * M_PI / 303750.0 / joint_gear_ratio_ * joint_inverse_;
                break;
            case PRO_H54_200_S500_R:
            case PRO_H54_100_S500_R:
                joint_pos_ = position * M_PI / 250961.5 / joint_gear_ratio_ * joint_inverse_;
                break;
            case PRO_H42_20_S300_R:
                joint_pos_ = position * M_PI / 151875.0 / joint_gear_ratio_ * joint_inverse_;
                break;
        }
    }

    if(groupBulkRead_->isAvailable(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_VELOCITY], 4))
    {
        int32_t velocity = groupBulkRead_->getData(motor_id_,
                DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_VELOCITY], 4);

        switch(dynamixel_series_)
        {
            case DynamixelSeries::SERIES_DYNAMIXEL_X:
                joint_vel_ = velocity * 0.229 * M_PI / 60.0 / joint_gear_ratio_ * joint_inverse_;
                break;
            case DynamixelSeries::SERIES_DYNAMIXEL_PRO:
                joint_vel_ = velocity * 0.00199234 * M_PI / 60.0 / joint_gear_ratio_ * joint_inverse_;
                break;
            case DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS:
                joint_vel_ = velocity * 0.01 * M_PI / 60.0 / joint_gear_ratio_ * joint_inverse_;
                break;
        }
    }

    int16_t read_current = 0;
    if(groupBulkRead_->isAvailable(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_CURRENT], 2))
    {
        read_current = groupBulkRead_->getData(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_CURRENT], 2);

        switch(dynamixel_series_) {
            case DynamixelSeries::SERIES_DYNAMIXEL_X:
                joint_eff_ = read_current * (2.69 / 1000.0) * joint_inverse_;
                break;
            case DynamixelSeries::SERIES_DYNAMIXEL_PRO:
                joint_eff_ = read_current * (16.11328 / 1000.0) * joint_inverse_;
                break;
            case DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS:
                joint_eff_ = read_current * (1.0 / 1000.0) * joint_inverse_;
                break;
        }
    }
}

void DynamixelMotor::execute_homing(const dynamixel_ros_control::HomingGoalConstPtr &goal)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dynamixel_ros_control::HomingFeedback feedback;
    dynamixel_ros_control::HomingResult result;
    bool success = true;

    ROS_INFO("[%s] homing mode: [%d], homing direction: [%d].", motor_name_.c_str(), homing_mode_, homing_direction_);

    switch(homing_mode_)
    {
        case 0: // current based homing
            {
                // 1. reboot
                ROS_INFO("[%s] homing: reboot for homing.", motor_name_.c_str());
                if(packetHandler_->reboot(portHandler_, motor_id_, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to reboot motor [%s]", motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }
                ros::Duration(0.25).sleep();

                // 2. set operating mode to velocity control
                ROS_INFO("[%s] set operating mode to velocity...", motor_name_.c_str());
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::OPERATING_MODE], 1, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set operating mode [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 3. torque_on
                ROS_INFO("[%s] torque enable...", motor_name_.c_str());
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::TORQUE_ENABLE], 1, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set torque enable [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 4. set goal current to
                ROS_INFO("[%s] set goal current to 1500mA", motor_name_.c_str());
                uint16_t goal_current = (uint16_t)(2.0 / DynamixelCurrentConvert[dynamixel_series_]);
                ROS_INFO("[%s] [%d]", motor_name_.c_str(), goal_current);
                if(packetHandler_->write2ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_CURRENT], goal_current, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set goal current [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 5. move homing direction with veloicty 1.5 rad/s (TBD)
                ROS_INFO("[%s] set goal velocity to 0.002 rad/s", motor_name_.c_str());
                int32_t goal_velocity = (int32_t)(0.004 / DynamixelVeolcityConvert[dynamixel_series_] * joint_gear_ratio_ * homing_direction_);
                ROS_INFO("[%s] [%d]", motor_name_.c_str(), goal_velocity);
                if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], goal_velocity, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 6. check moving status
                while(ros::ok())
                {
                    uint8_t moving = 0;
                    if(packetHandler_->read1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::MOVING], &moving, &dxl_error) != COMM_SUCCESS)
                    {
                        ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                    }

                    ROS_INFO("%d", moving);
                    if(moving == 0)
                    {
                        ROS_INFO("Stop");
                        if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], 0, &dxl_error) != COMM_SUCCESS)
                        {
                            ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                        }
                        break;
                    }
                }

                // 7. reboot
                ROS_INFO("[%s] homing: reboot for homing.", motor_name_.c_str());
                if(packetHandler_->reboot(portHandler_, motor_id_, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to reboot motor [%s]", motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }
                ros::Duration(0.25).sleep();

                // 8. torque_on
                ROS_INFO("[%s] torque enable...", motor_name_.c_str());
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::TORQUE_ENABLE], 1, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set torque enable [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                ros::Duration(0.1).sleep();

                // 9. Read current Position
                uint32_t current_position = 0;
                if(packetHandler_->read4ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_POSITION], &current_position, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                }


                ROS_INFO("=====  %f", (double)(current_position * M_PI / 501923.0 / joint_gear_ratio_ * joint_inverse_));
                init_and_ready(true);
                is_ready_ = true;

            }
            break;

        default:
        {
            ROS_WARN("[%s] can't support this homing mode...", motor_name_.c_str());
            ros::Duration(1).sleep();
        }
    }

    if(success)
    {
        result.done = true;
        ROS_INFO("[%s] homing completed.", motor_name_.c_str());
        homing_as_->setSucceeded(result);
    }
}

bool DynamixelMotor::write(double cmd)
{
    //ROS_INFO("[%s] write", motor_name_.c_str());

    int param_length = 0;
    uint8_t param_goal_value[4] = {0, 0, 0, 0};

    switch(operating_mode_)
    {
        case 0: // current
            {
                int16_t target_current = 0;
                target_current = (int16_t)(cmd / DynamixelCurrentConvert[dynamixel_series_] * joint_inverse_);
                ROS_INFO("[%s] %d %d", motor_name_.c_str(), target_current, DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_CURRENT]);

                param_length = 2;
                param_goal_value[0] = (uint8_t)(target_current >> 0);
                param_goal_value[1] = (uint8_t)(target_current >> 8);
                groupBulkWrite_->addParam(motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_CURRENT], param_length, param_goal_value);
            }
            break;
        case 1: // velocity
            {
                int32_t target_velocity = 0;
                target_velocity = cmd / DynamixelVeolcityConvert[dynamixel_series_] * joint_gear_ratio_ * joint_inverse_;

                param_length = 4;
                param_goal_value[0] = DXL_LOBYTE(DXL_LOWORD(target_velocity));
                param_goal_value[1] = DXL_HIBYTE(DXL_LOWORD(target_velocity));
                param_goal_value[2] = DXL_LOBYTE(DXL_HIWORD(target_velocity));
                param_goal_value[3] = DXL_HIBYTE(DXL_HIWORD(target_velocity));
                groupBulkWrite_->addParam(motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], param_length, param_goal_value);
            }
            break;
        case 3: // position
        case 4: // ext. position
            {
                int32_t target_position = 0;
                target_position = cmd / DynamixelPositionConvert[motor_model_num_] * joint_gear_ratio_ * joint_inverse_;

                param_length = 4;
                param_goal_value[0] = DXL_LOBYTE(DXL_LOWORD(target_position));
                param_goal_value[1] = DXL_HIBYTE(DXL_LOWORD(target_position));
                param_goal_value[2] = DXL_LOBYTE(DXL_HIWORD(target_position));
                param_goal_value[3] = DXL_HIBYTE(DXL_HIWORD(target_position));
                groupBulkWrite_->addParam(motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_POSITION], param_length, param_goal_value);
            }
            break;
    }
}

void DynamixelMotor::stop()
{
    uint8_t dxl_error = 0;
    if(packetHandler_->reboot(portHandler_, motor_id_, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to reboot motor [%s]", motor_name_.c_str());
    }
}

std::string DynamixelMotor::get_joint_name()
{
    return joint_name_;
}

void DynamixelMotor::get_current_value(double &pos, double &vel, double &effort)
{
    pos = joint_pos_;
    vel = joint_vel_;
    effort = joint_eff_;
}


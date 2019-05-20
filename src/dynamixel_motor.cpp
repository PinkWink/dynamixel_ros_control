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
}

DynamixelMotor::~DynamixelMotor()
{
}

bool DynamixelMotor::init(int model_number, std::string name)
{
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

    if(!groupBulkRead_->addParam(motor_id_,
            DynamixelReadStartAddress[dynamixel_series_], DynamixelReadLength[dynamixel_series_]))
    {
        ROS_ERROR("Failed to addParam position");
    }
}

bool DynamixelMotor::update()
{
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
                joint_eff_ = read_current * 2.69 / 1000.0 * joint_inverse_;
                break;
            case DynamixelSeries::SERIES_DYNAMIXEL_PRO:
                joint_eff_ = read_current * 16.11328 / 1000.0 * joint_inverse_;
                break;
            case DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS:
                joint_eff_ = read_current * 1.0 / 1000.0 * joint_inverse_;
                break;
        }
    }
}

bool DynamixelMotor::write(double cmd)
{
    int param_length = 0;
    uint8_t param_goal_value[4] = {0, 0, 0, 0};

    switch(operating_mode_)
    {
        case 0: // current
            {
                int16_t target_current = 0;
                switch(dynamixel_series_)
                {
                    case DynamixelSeries::SERIES_DYNAMIXEL_X:
                        target_current = (int16_t)(cmd / (2.69 / 1000.0) * joint_inverse_);
                        break;
                    case DynamixelSeries::SERIES_DYNAMIXEL_PRO:
                        target_current = (int16_t)(cmd / (16.11328 / 1000.0) * joint_inverse_);
                        break;
                    case DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS:
                        target_current = (int16_t)(cmd / (1.0 / 1000.0) * joint_inverse_);
                        break;
                }

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
                switch(dynamixel_series_)
                {
                    case DynamixelSeries::SERIES_DYNAMIXEL_X:
                        target_velocity = cmd / (0.229 * M_PI / 60.0) * joint_gear_ratio_ * joint_inverse_;
                        break;
                    case DynamixelSeries::SERIES_DYNAMIXEL_PRO:
                        target_velocity = cmd / (0.00199234 * M_PI / 60.0) * joint_gear_ratio_ * joint_inverse_;
                        break;
                    case DynamixelSeries::SERIES_DYNAMIXEL_PRO_PLUS:
                        target_velocity = cmd / (0.01 * M_PI / 60.0) * joint_gear_ratio_ * joint_inverse_;
                        break;
                }

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
                switch(motor_model_num_)
                {
                    case PRO_PLUS_H54P_200_S500_R:
                    case PRO_PLUS_H54P_100_S500_R:
                        target_position = cmd / (M_PI / 501923.0) * joint_gear_ratio_ * joint_inverse_;
                        break;
                    case PRO_PLUS_H42P_020_S300_R:
                        target_position = cmd / (M_PI / 303750.0) * joint_gear_ratio_ * joint_inverse_;
                        break;
                    case PRO_H54_200_S500_R:
                    case PRO_H54_100_S500_R:
                        target_position = cmd / (M_PI / 250961.5) * joint_gear_ratio_ * joint_inverse_;
                        break;
                    case PRO_H42_20_S300_R:
                        target_position = cmd / (M_PI / 151875.0) * joint_gear_ratio_ * joint_inverse_;
                        break;
                }

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


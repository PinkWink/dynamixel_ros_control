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
    velocity_p_gain_ = 0;
    velocity_i_gain_ = 0;

    is_ready_ = true;
    is_homing_ = false;
    homing_offset_ = 0.0;

    is_gripper_ = false;
}

DynamixelMotor::~DynamixelMotor()
{
}

bool DynamixelMotor::init(ros::NodeHandle& nh, ros::NodeHandle& pnh, int model_number, std::string name)
{
    motor_name_ = name;
    motor_model_num_ = model_number;
    dynamixel_series_ = DynamixelModel[motor_model_num_];

    pnh.getParam(name + "/id", motor_id_);
    pnh.getParam(name + "/operating_mode", operating_mode_);
    pnh.getParam(name + "/joint_name", joint_name_);
    pnh.getParam(name + "/gear_ratio", joint_gear_ratio_);
    pnh.getParam(name + "/inverse", joint_inverse_);
    pnh.getParam(name + "/profile_acceleration", profile_acceleration_);
    pnh.getParam(name + "/profile_velocity", profile_velocity_);
    pnh.param<int>(name + "/origin_offset", origin_offset_, 0);
    pnh.param<int>(name + "/velocity_gains/p", velocity_p_gain_, -1);
    pnh.param<int>(name + "/velocity_gains/i", velocity_i_gain_, -1);


    pnh.param<bool>(name + "/homing/enable", need_homing_, false);
    if(need_homing_)
    {
        if(!pnh.getParam(name + "/homing/mode", homing_mode_))
        {
            ROS_ERROR("[%s] please set parameter homing/mode.", motor_name_.c_str());
            return false;
        }
        pnh.param<double>(name + "/homing/direction", homing_direction_, 1.0);
        pnh.param<double>(name + "/homing/max_speed", homing_max_speed_, 0.002);
        pnh.param<double>(name + "/homing/current_limit", homing_current_limit_, 2.0);

        ROS_INFO("[%s] motor need homing.", motor_name_.c_str());
        is_homing_ = false;
        homing_as_ = boost::make_shared<actionlib::SimpleActionServer<dynamixel_ros_control::HomingAction>>(nh, name + "/homing", boost::bind(&DynamixelMotor::execute_homing, this, _1), false);
        homing_as_->start();

        is_ready_ = false;
    }

    pnh.param<bool>(name + "/gripper/enable", is_gripper_, false);
    if(is_gripper_)
    {
        pnh.param<double>(name + "/gripper/gap_size", gripper_gap_size_, 0.0);
        pnh.param<double>(name + "/gripper/current_limit", gripper_current_limit_, 1.0);
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

    if(velocity_i_gain_ != -1)
    {
        if(packetHandler_->write2ByteTxRx(portHandler_, motor_id_,
                DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::VELOCITY_I_GAIN], velocity_i_gain_, &dxl_error) != COMM_SUCCESS)
        {
            ROS_ERROR("Failed to set velocity_i_gain [%d] on [%s].", motor_id_, motor_name_.c_str());
            return false;
        }
        ROS_INFO("[%s] set velocity i gain to [%d].", motor_name_.c_str(), velocity_i_gain_);
    }

    if(velocity_p_gain_ != -1)
    {
        if(packetHandler_->write2ByteTxRx(portHandler_, motor_id_,
                DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::VELOCITY_P_GAIN], velocity_p_gain_, &dxl_error) != COMM_SUCCESS)
        {
            ROS_ERROR("Failed to set velocity_p_gain [%d] on [%s].", motor_id_, motor_name_.c_str());
            return false;
        }
        ROS_INFO("[%s] set velocity p gain to [%d].", motor_name_.c_str(), velocity_p_gain_);
    }
    // torque enable
    if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_, DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::TORQUE_ENABLE], 1, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set torque enable [%d] on [%s].", motor_id_, motor_name_.c_str());
        return false;
    }

    // set profile_accelration
    uint32_t target_acceleration = profile_acceleration_ / DynamixelVeolcityConvert[dynamixel_series_] * joint_gear_ratio_;
    if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_, DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PROFILE_ACCELERATION], target_acceleration, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set profile acceleration [%d] on [%s].", motor_id_, motor_name_.c_str());
        return false;
    }

    // set velocity limit
    uint32_t target_profile_velocity = profile_velocity_ / DynamixelVeolcityConvert[dynamixel_series_] * joint_gear_ratio_;
    if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_, DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PROFILE_VELOCITY], target_profile_velocity, &dxl_error) != COMM_SUCCESS)
    {
        ROS_ERROR("Failed to set profile velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
        return false;
    }

    if(is_gripper_)
    {
        uint16_t current_limit = 0;
        current_limit = (uint16_t)(gripper_current_limit_ / DynamixelCurrentConvert[dynamixel_series_]);
        if(packetHandler_->write2ByteTxRx(portHandler_, motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_CURRENT], current_limit, &dxl_error) != COMM_SUCCESS)
        {
            ROS_ERROR("Failed to set operating mode [%d] on [%s].", motor_id_, motor_name_.c_str());
            return false;
        }
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
    // present position
    if(groupBulkRead_->isAvailable(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_POSITION], 4))
    {
        int32_t position = groupBulkRead_->getData(motor_id_,
                DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_POSITION], 4);
        position = position - origin_offset_;

        joint_pos_ = (position * DynamixelPositionConvert[motor_model_num_] / joint_gear_ratio_ * joint_inverse_) - homing_offset_;

        if(is_gripper_)
        {
            joint_pos_ = (gripper_gap_size_ - joint_pos_) / gripper_gap_size_;
        }
    }

    // present velocity
    if(groupBulkRead_->isAvailable(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_VELOCITY], 4))
    {
        int32_t velocity = groupBulkRead_->getData(motor_id_,
                DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_VELOCITY], 4);

        joint_vel_ = velocity * DynamixelVeolcityConvert[dynamixel_series_] / joint_gear_ratio_ * joint_inverse_;
    }

    // present current
    int16_t read_current = 0;
    if(groupBulkRead_->isAvailable(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_CURRENT], 2))
    {
        read_current = groupBulkRead_->getData(motor_id_,
            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_CURRENT], 2);

        joint_eff_ = read_current * DynamixelCurrentConvert[dynamixel_series_] * joint_inverse_;
    }
}

void DynamixelMotor::execute_homing(const dynamixel_ros_control::HomingGoalConstPtr &goal)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dynamixel_ros_control::HomingFeedback feedback;
    dynamixel_ros_control::HomingResult result;
    bool success = true;

    ROS_INFO("[%s] homing mode: [%d], homing direction: [%.1f].", motor_name_.c_str(), homing_mode_, homing_direction_);

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
                ros::Duration(0.3).sleep();

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
                ROS_INFO("[%s] set goal current to %.3f (mA)...", motor_name_.c_str(), homing_current_limit_ * 1000.0);
                uint16_t goal_current = (uint16_t)(homing_current_limit_ / DynamixelCurrentConvert[dynamixel_series_]);
                if(packetHandler_->write2ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_CURRENT], goal_current, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set goal current [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 5. move homing direction with veloicty  (TBD)
                ROS_INFO("[%s] set goal velocity to %.3f rad/s", motor_name_.c_str(), homing_max_speed_);
                int32_t goal_velocity = (int32_t)(homing_max_speed_ / DynamixelVeolcityConvert[dynamixel_series_] * joint_gear_ratio_ * homing_direction_);
                if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], goal_velocity, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                ros::Duration(0.06).sleep();

                // 6. move! and check moving status
                while(ros::ok())
                {
                    uint8_t moving = 0;
                    if(packetHandler_->read1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::MOVING], &moving, &dxl_error) != COMM_SUCCESS)
                    {
                        ROS_ERROR("Failed to get moving status [%d] on [%s].", motor_id_, motor_name_.c_str());
                    }
                    if(moving == 0)
                    {
                        ROS_INFO("[%s] limit detected...", motor_name_.c_str());
                        if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], 0, &dxl_error) != COMM_SUCCESS)
                        {
                            ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                        }
                        break;
                    }

                    ros::Duration(0.0).sleep();
                }

                // 7. reboot for clear old position
                ROS_INFO("[%s] homing: reboot for clear registers.", motor_name_.c_str());
                if(packetHandler_->reboot(portHandler_, motor_id_, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to reboot motor [%s]", motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }
                ros::Duration(0.4).sleep();

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
                uint32_t position = 0;
                if(packetHandler_->read4ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::PRESENT_POSITION], &position, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to get present position [%d] on [%s].", motor_id_, motor_name_.c_str());
                }

                int32_t sign_position = (int32_t)position;
                homing_offset_ = (double)(sign_position * DynamixelPositionConvert[motor_model_num_] / joint_gear_ratio_ * joint_inverse_);
                ROS_INFO("[%s] homing offset: %f", motor_name_.c_str(), homing_offset_);
                init_and_ready(true);
                is_ready_ = true;
            }
            break;

        case 1: // homing with digital input (limit sensor)
            {
                // 1. reboot
                ROS_INFO("[%s] homing: reboot for homing.", motor_name_.c_str());
                if(packetHandler_->reboot(portHandler_, motor_id_, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to reboot motor [%s]", motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }
                ros::Duration(0.3).sleep();

                // 2. set operating mode to velocity control
                ROS_INFO("[%s] set operating mode to velocity...", motor_name_.c_str());
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::OPERATING_MODE], 1, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set operating mode [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 3. set port mode to digital input pull down
                ROS_INFO("[%s] set port mode to digital input (pul down)...", motor_name_.c_str());
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::EXTERNAL_PORT_MODE_1], 3, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set port mode to digital input (pul down) [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::EXTERNAL_PORT_MODE_2], 3, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set port mode to digital input (pul down) [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::EXTERNAL_PORT_MODE_3], 3, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set port mode to digital input (pul down) [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::EXTERNAL_PORT_MODE_4], 3, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set port mode to digital input (pul down) [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 4. torque_on
                ROS_INFO("[%s] torque enable...", motor_name_.c_str());
                if(packetHandler_->write1ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::TORQUE_ENABLE], 1, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set torque enable [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                // 5. move homing direction with veloicty  (TBD)
                ROS_INFO("[%s] set goal velocity to %.3f rad/s", motor_name_.c_str(), homing_max_speed_);
                int32_t goal_velocity = (int32_t)(homing_max_speed_ / DynamixelVeolcityConvert[dynamixel_series_] * joint_gear_ratio_ * homing_direction_);
                if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], goal_velocity, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                ros::Duration(0.06).sleep();

                // 6. move! and check moving status
                uint8_t ext_port_data[4] = {0, };
                while(ros::ok())
                {
                    uint8_t port_data[8] = {0, };
                    if(packetHandler_->readTxRx(portHandler_, motor_id_,DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::EXTERNAL_PORT_DATA_1], 8,
                        port_data, &dxl_error) != COMM_SUCCESS)
                    {
                        ROS_ERROR("Failed to get external port data [%d] on [%s].", motor_id_, motor_name_.c_str());
                    }
                    // ROS_INFO("%d %d %d %d", port_data[0], port_data[2], port_data[4], port_data[6]);
                    if(port_data[0] || port_data[2] || port_data[4] || port_data[6])
                    {
                        ROS_INFO("[%s] limit detected...", motor_name_.c_str());
                        ext_port_data[0] = port_data[0];
                        ext_port_data[1] = port_data[2];
                        ext_port_data[2] = port_data[4];
                        ext_port_data[3] = port_data[6];

                        if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], 0, &dxl_error) != COMM_SUCCESS)
                        {
                            ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                            result.done = false;
                            homing_as_->setSucceeded(result);
                        }
                        break;
                    }
                    ros::Duration(0.0).sleep();
                }

                // 7. if port0 or port1 is detected -> just go to next step (reset), if port 2 or port3 is detected -> turn opposite direction and wait detect port 0
                if(ext_port_data[0] == 1 || ext_port_data[1] == 1)
                {
                    ROS_INFO("[%s] just go next step...", motor_name_.c_str());
                }
                else if(ext_port_data[2] == 1 || ext_port_data[3] == 1)
                {
                    ROS_INFO("[%s] joint is on other side, so move opposite direction...", motor_name_.c_str());
                    if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], -1 * goal_velocity, &dxl_error) != COMM_SUCCESS)
                    {
                        ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                        result.done = false;
                        homing_as_->setSucceeded(result);
                    }

                    while(ros::ok())
                    {
                        uint8_t port_data[8] = {0, };
                        if(packetHandler_->readTxRx(portHandler_, motor_id_,DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::EXTERNAL_PORT_DATA_1], 8,
                            port_data, &dxl_error) != COMM_SUCCESS)
                        {
                            ROS_ERROR("Failed to get external port data [%d] on [%s].", motor_id_, motor_name_.c_str());
                        }
                        if(port_data[0] == 1)
                        {
                            ROS_INFO("[%s] joint is on origin region...", motor_name_.c_str());

                            if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                                DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], 0, &dxl_error) != COMM_SUCCESS)
                            {
                                ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                                result.done = false;
                                homing_as_->setSucceeded(result);
                            }

                            break;
                        }
                    }
                }

                // 8. reset
                init_and_ready(true);

                // 9. move to origin position
                int32_t origin_position = origin_offset_;
                if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_POSITION], origin_position, &dxl_error) != COMM_SUCCESS)
                {
                    ROS_ERROR("Failed to set goal position [%d] on [%s].", motor_id_, motor_name_.c_str());
                    result.done = false;
                    homing_as_->setSucceeded(result);
                }

                ros::Duration(0.2).sleep();

                // 10. wait for stop moving
                while(ros::ok())
                {
                    uint8_t moving = 0;
                    if(packetHandler_->read1ByteTxRx(portHandler_, motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::MOVING], &moving, &dxl_error) != COMM_SUCCESS)
                    {
                        ROS_ERROR("Failed to get moving status [%d] on [%s].", motor_id_, motor_name_.c_str());
                    }
                    if(moving == 0)
                    {
                        ROS_INFO("[%s] origin reached...", motor_name_.c_str());
                        if(packetHandler_->write4ByteTxRx(portHandler_, motor_id_,
                            DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_VELOCITY], 0, &dxl_error) != COMM_SUCCESS)
                        {
                            ROS_ERROR("Failed to set goal velocity [%d] on [%s].", motor_id_, motor_name_.c_str());
                        }
                        break;
                    }
                    ros::Duration(0.0).sleep();
                }
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
    int param_length = 0;
    uint8_t param_goal_value[4] = {0, 0, 0, 0};

    switch(operating_mode_)
    {
        case 0: // current
            {
                int16_t target_current = 0;
                target_current = (int16_t)(cmd / DynamixelCurrentConvert[dynamixel_series_] * joint_inverse_);
                // ROS_INFO("[%s] %d %d", motor_name_.c_str(), target_current, DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_CURRENT]);

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
                cmd = cmd + homing_offset_;

                target_position = (cmd / DynamixelPositionConvert[motor_model_num_] * joint_gear_ratio_ * joint_inverse_);
                target_position = target_position + origin_offset_;

                // ROS_WARN("%s %d %d", motor_name_.c_str(), target_position,  DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_POSITION]);

                param_length = 4;
                param_goal_value[0] = DXL_LOBYTE(DXL_LOWORD(target_position));
                param_goal_value[1] = DXL_HIBYTE(DXL_LOWORD(target_position));
                param_goal_value[2] = DXL_LOBYTE(DXL_HIWORD(target_position));
                param_goal_value[3] = DXL_HIBYTE(DXL_HIWORD(target_position));
                groupBulkWrite_->addParam(motor_id_,
                    DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_POSITION], param_length, param_goal_value);
            }
            break;
        case 5: // gripper, current_based position control
            {
                int32_t target_position = 0;
                if(is_gripper_)
                {
                    target_position = (int32_t)((1.0 - cmd) * gripper_gap_size_);

                    param_length = 4;
                    param_goal_value[0] = DXL_LOBYTE(DXL_LOWORD(target_position));
                    param_goal_value[1] = DXL_HIBYTE(DXL_LOWORD(target_position));
                    param_goal_value[2] = DXL_LOBYTE(DXL_HIWORD(target_position));
                    param_goal_value[3] = DXL_HIBYTE(DXL_HIWORD(target_position));
                    groupBulkWrite_->addParam(motor_id_,
                        DynamixelControlTable[dynamixel_series_][DynamixelControlTableItem::GOAL_POSITION], param_length, param_goal_value);
                }
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


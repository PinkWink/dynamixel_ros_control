#include "dynamixel_ros_control/dynamixel_hw_interface.h"

DynamixelHWInterface::DynamixelHWInterface() {
    portHandler_ = NULL;
    packetHandler_ = NULL;
}

DynamixelHWInterface::~DynamixelHWInterface() {
    if(portHandler_ != NULL)
    {
        portHandler_->closePort();
    }
}

bool DynamixelHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
    // get port name
    std::string portName;
    if(!robot_hw_nh.getParam("port_name", portName))
    {
        ROS_ERROR("[%s] Failed to get port name. Please set the parameter ~port_name", ros::this_node::getName().c_str());
        return false;
    }

    portHandler_ = dynamixel::PortHandler::getPortHandler(portName.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
    groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_);

    if(!portHandler_->openPort())
    {
        ROS_ERROR("Failed to open port %s", portName.c_str());
        return false;
    }

    // get baudrate
    int baudrate;
    if(!robot_hw_nh.getParam("baudrate", baudrate)) {
        ROS_ERROR("Failed to get baudrate. Please set the parameter ~baudrate");
        return false;
    }

    if(!portHandler_->setBaudRate(baudrate))
    {
        ROS_ERROR("Failed to set baudrate %d", baudrate);
        return false;
    }

    // get motor list from config file (parameter server)
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    uint16_t dxl_model_number;

    std::vector<std::string> dynamixel_hw;
    if(!robot_hw_nh.getParam("dynamixel_hw", dynamixel_hw)) {
        ROS_ERROR("[%s] Failed to get dynamixel_hw list.", ros::this_node::getName().c_str());
        return false;
    }

    joint_cmd_.resize(dynamixel_hw.size());
    joint_pos_.resize(dynamixel_hw.size());
    joint_vel_.resize(dynamixel_hw.size());
    joint_eff_.resize(dynamixel_hw.size());

    // ping and regiter interface
    for(size_t i = 0; i < dynamixel_hw.size(); i++)
    {
        int id = 0;
        if(!robot_hw_nh.getParam(dynamixel_hw[i] + "/id", id))
        {
            ROS_ERROR("[%s] Failed to get id from config file.", ros::this_node::getName().c_str());
            return false;
        }

        dxl_comm_result = packetHandler_->ping(portHandler_, id, &dxl_model_number, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS)
        {
            ROS_ERROR("[%s] Communicaton Failed. %s", ros::this_node::getName().c_str(), packetHandler_->getTxRxResult(dxl_comm_result));
            return false;
        }
        if(dxl_error != 0)
        {
            ROS_ERROR("[%s] DXL Error. %s", ros::this_node::getName().c_str(), packetHandler_->getRxPacketError(dxl_error));
            return false;
        }

        // save to vector
        DynamixelMotor *motor = new DynamixelMotor(portHandler_, packetHandler_, groupBulkRead_, groupBulkWrite_);
        motor->init(dxl_model_number, dynamixel_hw[i]);
        dynamixel_motors_.push_back(motor);

        hardware_interface::JointStateHandle state_handle(motor->get_joint_name(), &joint_pos_[i], &joint_vel_[i], &joint_eff_[i]);
        jnt_state_interface_.registerHandle(state_handle);

        ROS_INFO("[%s] Found dynamixel motors on [%s]: ID [%d], Model Number [%s].", ros::this_node::getName().c_str(), dynamixel_hw[i].c_str(), (int)id, dynamixel_model_name[dxl_model_number].c_str());
    }

    registerInterface(&jnt_state_interface_);
    ROS_INFO("[%s] Initialized...", ros::this_node::getName().c_str());
    return true;
}

void DynamixelHWInterface::read(const ros::Time& time, const ros::Duration& period)
{
    groupBulkRead_->txRxPacket();
    for(size_t i = 0; i < dynamixel_motors_.size(); i++)
    {
        dynamixel_motors_[i]->update();
        dynamixel_motors_[i]->get_current_value(joint_pos_[i], joint_vel_[i], joint_eff_[i]);
    }
}

void DynamixelHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
    // std::map<int, DynamixelMotor*>::iterator it = mapDynamixelMotors_.begin();
    // while(it != mapDynamixelMotors_.end())
    // {
    //     it->second->write();
    //     it++;
    // }

    // groupBulkWrite_->txPacket();
    // groupBulkWrite_->clearParam();
}

bool DynamixelHWInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    return true;
}

void DynamixelHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
}
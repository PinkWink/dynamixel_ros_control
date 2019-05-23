#include "dynamixel_ros_control/dynamixel_hw_interface.h"

DynamixelHWInterface::DynamixelHWInterface() {
    portHandler_ = NULL;
    packetHandler_ = NULL;
}

DynamixelHWInterface::~DynamixelHWInterface() {
    if(portHandler_ != NULL)
    {
        for(size_t i = 0; i < dynamixel_motors_.size(); i++)
        {
            dynamixel_motors_[i]->stop();
        }
        portHandler_->closePort();
    }
}

bool DynamixelHWInterface::is_ready()
{
    bool result = true;
    for(size_t i = 0; i < dynamixel_motors_.size(); i++)
    {
        result = result && dynamixel_motors_[i]->is_ready();
    }

    return result;
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
    for(size_t i = 0; i < joint_cmd_.size(); i++)
    {
        joint_cmd_[i] = std::nan("0");
    }
    joint_pos_.resize(dynamixel_hw.size());
    joint_vel_.resize(dynamixel_hw.size());
    joint_eff_.resize(dynamixel_hw.size());

    // ping and regiter interface
    for(size_t i = 0; i < dynamixel_hw.size(); i++)
    {
        int id = 0;
        int operating_mode = 0;
        if(!robot_hw_nh.getParam(dynamixel_hw[i] + "/id", id))
        {
            ROS_ERROR("[%s] Failed to get id from config file.", ros::this_node::getName().c_str());
            return false;
        }

        if(!robot_hw_nh.getParam(dynamixel_hw[i] + "/operating_mode", operating_mode))
        {
            ROS_ERROR("[%s] Failed to get operating_mode from config file.", ros::this_node::getName().c_str());
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

        // save to std::vector
        DynamixelMotor *motor = new DynamixelMotor(portHandler_, packetHandler_, groupBulkRead_, groupBulkWrite_);
        ROS_INFO("[%s] Found dynamixel motors on [%s]: ID [%d], Model Number [%s].",
            ros::this_node::getName().c_str(), dynamixel_hw[i].c_str(), (int)id, dynamixel_model_name[dxl_model_number].c_str());
        motor->init(dxl_model_number, dynamixel_hw[i]);
        dynamixel_motors_.push_back(motor);

        // joint_state
        hardware_interface::JointStateHandle state_handle(motor->get_joint_name(), &joint_pos_[i], &joint_vel_[i], &joint_eff_[i]);
        jnt_state_interface_.registerHandle(state_handle);

        // joint_cmd
        switch(operating_mode)
        {
            case 0: // current
                {
                    hardware_interface::JointHandle eff_handle(jnt_state_interface_.getHandle(motor->get_joint_name()), &joint_cmd_[i]);
                    jnt_eff_interface_.registerHandle(eff_handle);
                    break;
                }
            case 1: // velocity
                {
                    hardware_interface::JointHandle vel_handle(jnt_state_interface_.getHandle(motor->get_joint_name()), &joint_cmd_[i]);
                    jnt_vel_interface_.registerHandle(vel_handle);
                    break;
                }
            case 3: // position
            case 4: // ext. position
            case 5: // current based position
                {
                    hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(motor->get_joint_name()), &joint_cmd_[i]);
                    jnt_pos_interface_.registerHandle(pos_handle);
                    break;
                }
            default:
                {
                    ROS_ERROR("Wrong operating_mode. Check again.");
                    break;
                }
        }
    }

    registerInterface(&jnt_eff_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_pos_interface_);
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
    for(size_t i = 0; i < dynamixel_motors_.size(); i++)
    {
        if(!(std::isnan(joint_cmd_[i])))
        {
            dynamixel_motors_[i]->write(joint_cmd_[i]);
        }
    }

    groupBulkWrite_->txPacket();
    groupBulkWrite_->clearParam();
}

bool DynamixelHWInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                ROS_ERROR_STREAM("Bad interface: " << res_it->hardware_interface);
                std::cout << res_it->hardware_interface;
                return false;
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    ROS_ERROR_STREAM("Bad resource: " << (*ctrl_res));
                    std::cout << (*ctrl_res);
                    return false;
                }
            }
        }
    }
    return true;
}

void DynamixelHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                throw hardware_interface::HardwareInterfaceException("Hardware_interface " + res_it->hardware_interface + " is not registered");
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    throw hardware_interface::HardwareInterfaceException("Resource " + *ctrl_res + " is not registered");
                }
            }
        }
    }
}
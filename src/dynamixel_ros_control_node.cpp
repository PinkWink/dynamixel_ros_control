#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamixel_ros_control/dynamixel_hw_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_ros_control_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double control_frequency = 0.0;
    pnh.param<double>("rate", control_frequency, 100.0);

    DynamixelHWInterface dynamixels;
    if(!dynamixels.init(nh, pnh))
    {
        return -1;
    }

    ROS_INFO("[%s] wait for ready dynamixels...", ros::this_node::getName().c_str());
    while(ros::ok() && !dynamixels.is_ready())
        ros::Duration(0.01).sleep();

    ros::Duration(1.0).sleep();

    ROS_INFO("[%s] ready. start controller...", ros::this_node::getName().c_str());
    controller_manager::ControllerManager cm(&dynamixels, nh);
    ros::Duration period(1.0/control_frequency);

    while(ros::ok())
    {
        if(dynamixels.is_ready())
            dynamixels.read(ros::Time::now(), period);

        cm.update(ros::Time::now(), period);

        if(dynamixels.is_ready())
            dynamixels.write(ros::Time::now(), period);
        period.sleep();
    }

    return 0;
}
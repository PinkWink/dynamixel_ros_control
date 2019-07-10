#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamixel_ros_control/dynamixel_hw_interface.h>


class DynamixelROSControlNode
{
    public:
        DynamixelROSControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            nh_ = nh;
            pnh_ = pnh;

            double control_frequency = 0.0;
            pnh_.param<double>("rate", control_frequency, 100.0);

            dynamixels = boost::make_shared<DynamixelHWInterface>();
            assert(dynamixels->init(nh_, pnh_));

            ROS_INFO("[%s] wait for ready dynamixels...", ros::this_node::getName().c_str());
            while(ros::ok() && !dynamixels->is_ready())
            {
                ros::spinOnce();
                ros::Duration(0.01).sleep();
            }

            ros::Duration(1.0).sleep();
            ROS_INFO("[%s] ready. start controller...", ros::this_node::getName().c_str());

            cm = boost::make_shared<controller_manager::ControllerManager>((hardware_interface::RobotHW*)&dynamixels, nh_);
            period = ros::Duration(1.0/control_frequency);

            loop_timer = nh_.createTimer(period, &DynamixelROSControlNode::callback, this);
            loop_timer.start();
        }
        ~DynamixelROSControlNode()
        {
            loop_timer.stop();
        }

    private:
        void callback(const ros::TimerEvent& event)
        {
            if(dynamixels->is_ready())
            {
                dynamixels->read(ros::Time::now(), period);
            }

            cm->update(ros::Time::now(), period);

            if(dynamixels->is_ready())
            {
                dynamixels->write(ros::Time::now(), period);
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        boost::shared_ptr<DynamixelHWInterface> dynamixels;
        boost::shared_ptr<controller_manager::ControllerManager> cm;
        ros::Duration period;
        ros::Timer loop_timer;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_ros_control_node");
    ros::AsyncSpinner spinner(2);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    DynamixelROSControlNode m(nh, pnh);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
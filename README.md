# dynamixel_ros_control

For control the Dynamixel Series Motors with ros_control


## Installation

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/byeongkyu/dynamixel_ros_control.git
    $ rosdep install --from-paths ./dynamixel_ros_control --ignore-src -r -y
    $ catkin build


## Usage

### Create a package for your robot (for example open_manipulator)

    $ cd ~/catkin_ws/src
    $ catkin create pkg open_manipulator_with_ros_control --catkin-deps dynamixel_ros_control

### Create configuration file for dynamixel hardware interface

    $ roscd open_manipulator_with_ros_control
    $ mkdir config
    $ vi manipulator_conf.yaml

Edit yaml file as show below

``` yaml
dynamixel_hw:
  - joint1_motor
  - joint2_motor
  - joint3_motor
  - joint4_motor
  - gripper_motor

joint1_motor:
  id: 11
  operating_mode: 3
  joint_name: joint1
  gear_ratio: 1.0
  inverse: 1.0
  profile_acceleration: 0.6
  profile_velocity: 0.6
  origin_offset: 2048

joint2_motor:
  id: 12
  operating_mode: 3
  joint_name: joint2
  gear_ratio: 1.0
  inverse: 1.0
  profile_acceleration: 0.6
  profile_velocity: 0.6
  origin_offset: 2048

joint3_motor:
  id: 13
  operating_mode: 3
  joint_name: joint3
  gear_ratio: 1.0
  inverse: 1.0
  profile_acceleration: 0.6
  profile_velocity: 0.6
  origin_offset: 2048

joint4_motor:
  id: 14
  operating_mode: 3
  joint_name: joint4
  gear_ratio: 1.0
  inverse: 1.0
  profile_acceleration: 0.6
  profile_velocity: 0.6
  origin_offset: 2048

gripper_motor:
  id: 15
  operating_mode: 5
  joint_name: gripper
  gear_ratio: 1.0
  inverse: 1.0
  profile_acceleration: 0
  profile_velocity: 0.6
  origin_offset: 2635
  gripper:
    enable: true
    gap_size: 1616
    current_limit: 0.1
    stroke: 0.056
```

### Create a configuration file for controllers

    $ vi controllers.yaml

Edit yaml file as show below (typical ros_control configuration file):

``` yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 25

arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
  constraints:
    goal_time: 30.0
    stopped_velocity_tolerance: 0.05
    joint1: {trajectory: 0.1, goal: 0.1}
    joint2: {trajectory: 0.1, goal: 0.1}
    joint3: {trajectory: 0.1, goal: 0.1}
    joint4: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10

gripper_controller:
  type: position_controllers/JointPositionController
  joint: gripper
```

### Create a launch file for loading hardware and controllers

    $ cd launch
    $ vi bringup_hw.launch

``` xml
<launch>
    <arg name="port" default="/dev/ttyUSB0"/>
    <arg name="baudrate" default="1000000"/>
    <arg name="rate" default="25"/>

    <arg name="manipulator_config" default="$(find open_manipulator_with_ros_control)/config/manipulator_conf.yaml"/>

    <node name="manipulator_dynamixel_node" type="dynamixel_ros_control_node" pkg="dynamixel_ros_control" output="screen" required="true">
        <rosparam file="$(arg manipulator_config)" command="load"/>
        <param name="port_name" value="$(arg port)"/>
        <param name="baudrate" value="$(arg baudrate)"/>
        <param name="rate" value="$(arg rate)"/>
    </node>
</launch>
```

    $ vi bringup_controllers.launch

``` xml
<launch>
    <rosparam file="$(find open_manipulator_with_ros_control)/config/controllers.yaml" command="load"/>
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="true"
                output="screen"   args="joint_state_controller
                                        arm_controller
                                        gripper_controller"/>
</launch>
```

    $ vi bringup.launch

``` xml
<launch>
    <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find open_manipulator_description)/urdf//open_manipulator.urdf.xacro'"/>
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
</launch>
```

## Execute the launch

    $ roslaunch open_manipulator_with_ros_control bringup.launch
    $ roslaunch open_manipulator_with_ros_control bringup_hw.launch
    $ roslaunch open_manipulator_with_ros_control bringup_controller.launch

## For control joints

    $ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
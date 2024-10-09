![image](https://github.com/user-attachments/assets/6426a0ee-569b-4ba9-98a8-d80032ab8681)# Marty-Gazebo
## ros-to-real

## Jump_control.cpp

  #include <ros/ros.h>
  #include <unitree_legged_msgs/LowCmd.h>
  #include <unitree_legged_msgs/LowState.h>
  #include "unitree_legged_sdk/unitree_legged_sdk.h"
  #include "convert.h"
  #include <thread>
  #include <chrono>
  #include <vector>
  
  using namespace UNITREE_LEGGED_SDK;

  class JumpControl {
  public:
      JumpControl(ros::NodeHandle& nh);
      void run();
  
  private:
      ros::NodeHandle& nh_;
      ros::Publisher cmd_pub_;
      ros::Subscriber state_sub_;
      unitree_legged_msgs::LowCmd low_cmd_;
      unitree_legged_msgs::LowState low_state_;
  
      void stateCallback(const unitree_legged_msgs::LowState::ConstPtr& msg);
      void initCommand();
      void executeJump();
  };

    JumpControl::JumpControl(ros::NodeHandle& nh) : nh_(nh) {
      cmd_pub_ = nh_.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
      state_sub_ = nh_.subscribe("low_state", 1, &JumpControl::stateCallback, this);
      initCommand();
  }
  
  void JumpControl::stateCallback(const unitree_legged_msgs::LowState::ConstPtr& msg) {
      low_state_ = *msg;
  }
  
  void JumpControl::initCommand() {
      low_cmd_.head[0] = 0xFE;
      low_cmd_.head[1] = 0xEF;
      low_cmd_.levelFlag = LOWLEVEL;
  
      for (int i = 0; i < 12; i++) {
          low_cmd_.motorCmd[i].mode = 0x0A;
          low_cmd_.motorCmd[i].q = PosStopF;
          low_cmd_.motorCmd[i].Kp = 0;
          low_cmd_.motorCmd[i].dq = VelStopF;
          low_cmd_.motorCmd[i].Kd = 0;
          low_cmd_.motorCmd[i].tau = 0;
      }
  }
  
  void JumpControl::executeJump() {
      // Implement your jumping logic here, similar to what you have in body.cpp
      // Use low_cmd_ to send commands and low_state_ to read the current state
  }
  
  void JumpControl::run() {
      ros::Rate rate(500);  // 500 Hz control loop
      while (ros::ok()) {
          executeJump();
          cmd_pub_.publish(low_cmd_);
          ros::spinOnce();
          rate.sleep();
      }
  }

## launch

    <launch>
        <arg name="ctrl_level" default="lowlevel"/>
    
        <node pkg="unitree_legged_real" type="ros_udp" name="node_ros_udp" output="screen" args="$(arg ctrl_level)"/>
    
        <node pkg="your_package_name" type="jump_control" name="jump_control" output="screen"/>
    
        <param name="control_level" value="$(arg ctrl_level)"/>
    </launch>

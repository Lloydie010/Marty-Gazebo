# Marty-Gazebo
## ros-to-real

## jumpcontrl altogether

    #include <ros/ros.h>
    #include <unitree_legged_msgs/LowCmd.h>
    #include <unitree_legged_msgs/LowState.h>
    #include "unitree_legged_sdk/unitree_legged_sdk.h"
    #include "convert.h"
    #include <vector>
    #include <string>
    #include <fstream>
    #include <sstream>
    #include <iostream>
    
    using namespace UNITREE_LEGGED_SDK;
    
    class JumpControl {
    public:
        JumpControl() : loop_rate(500) {
            pub = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
            sub = nh.subscribe("low_state", 1, &JumpControl::stateCallback, this);
            initCommand();
        }
    
        bool loadTrajectoryVel(const std::string& filename, std::vector<std::vector<double>>& trajectory) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return false;
            }
    
            std::string line;
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                std::vector<double> row;
                std::string value;
                while (std::getline(iss, value, ',')) {
                    row.push_back(std::stod(value));
                }
                trajectory.push_back(row);
            }
            return true;
        }
    
        void moveAllPosInt(const std::vector<double>& targetPos, double duration) {
            std::vector<double> lastPos(12);
            for (int j = 0; j < 12; j++) {
                lastPos[j] = low_state.motorState[j].q;
            }
    
            for (int i = 1; i <= duration; i++) {
                double percent = static_cast<double>(i) / duration;
                for (int j = 0; j < 12; j++) {
                    low_cmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
                    low_cmd.motorCmd[j].dq = 0;
                    low_cmd.motorCmd[j].Kp = 5.0;
                    low_cmd.motorCmd[j].Kd = 1.0;
                    low_cmd.motorCmd[j].tau = 0;
                }
                sendCommand();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
    
        void run() {
            std::vector<std::vector<double>> trajectory;
            if (!loadTrajectoryVel("path_to_your_csv_file.csv", trajectory)) {
                return;
            }
    
            std::cout << "Trajectory loaded. Press Enter to start the jump..." << std::endl;
            std::cin.ignore();
    
            for (const auto& pos : trajectory) {
                moveAllPosInt(pos, 10);  // Adjust the duration as needed
            }
        }
    
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Rate loop_rate;
        unitree_legged_msgs::LowCmd low_cmd;
        unitree_legged_msgs::LowState low_state;
    
        void stateCallback(const unitree_legged_msgs::LowState::ConstPtr& msg) {
            low_state = *msg;
        }
    
        void initCommand() {
            low_cmd.head[0] = 0xFE;
            low_cmd.head[1] = 0xEF;
            low_cmd.levelFlag = LOWLEVEL;
    
            for (int i = 0; i < 12; i++) {
                low_cmd.motorCmd[i].mode = 0x0A;
                low_cmd.motorCmd[i].q = PosStopF;
                low_cmd.motorCmd[i].Kp = 0;
                low_cmd.motorCmd[i].dq = VelStopF;
                low_cmd.motorCmd[i].Kd = 0;
                low_cmd.motorCmd[i].tau = 0;
            }
        }
    
        void sendCommand() {
            pub.publish(low_cmd);
        }
    };
    
    int main(int argc, char **argv) {
        ros::init(argc, argv, "jump_control");
    
        std::cout << "Communication level is set to LOW-level." << std::endl
                  << "WARNING: Make sure the robot is hung up." << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
    
        JumpControl jumpControl;
        jumpControl.run();
    
        return 0;
    }

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

    int main(int argc, char** argv) {
        ros::init(argc, argv, "jump_control");
        ros::NodeHandle nh;
    
        JumpControl jump_control(nh);
        jump_control.run();
    
        return 0;
    }

## launch

    <launch>
        <arg name="ctrl_level" default="lowlevel"/>
    
        <node pkg="unitree_legged_real" type="ros_udp" name="node_ros_udp" output="screen" args="$(arg ctrl_level)"/>
    
        <node pkg="your_package_name" type="jump_control" name="jump_control" output="screen"/>
    
        <param name="control_level" value="$(arg ctrl_level)"/>
    </launch>

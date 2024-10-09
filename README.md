# Marty-Gazebo
## ros-to-real

## UDP code

    #include <ros/ros.h>
    #include <unitree_legged_msgs/LowCmd.h>
    #include <unitree_legged_msgs/LowState.h>
    #include "unitree_legged_sdk/unitree_legged_sdk.h"
    #include "convert.h"
    #include <fstream>
    #include <sstream>
    #include <vector>
    #include <string>
    #include <cmath>
    
    using namespace UNITREE_LEGGED_SDK;
    
    class TrajectoryControl
    {
    public:
        TrajectoryControl() : safe(LeggedType::Go1), udp(LOWLEVEL)
        {
            udp.InitCmdData(lowCmd);
        }
    
        void UDPRecv()
        {
            udp.Recv();
        }
    
        void UDPSend()
        {
            udp.Send();
        }
    
        void RobotControl() 
        {
            motiontime++;
    
            udp.GetRecv(lowState);
            
            // State
            printf("[%d] Received %f %f %f %f\n", motiontime, 
                   lowState.motorState[FR_0].q, lowState.motorState[FR_0].dq,
                   lowState.motorState[FR_0].tauEst, lowState.motorState[FR_1].q);
    
            // Trajectory control
            if (motiontime > 10 && !trajectoryStarted)
            {
                trajectoryStarted = true;
                startTrajectory();
            }
    
            // Send command to robot
            udp.SetSend(lowCmd);
        }
    
        void startTrajectory()
        {
            std::string filename = "trajectory.csv";  // Update with your CSV file path
            std::vector<std::vector<double>> positions;
            std::vector<std::vector<double>> velocities;
            std::vector<double> durations;
    
            if (loadTrajectoryVel(filename, positions, velocities, durations))
            {
                moveAllPosVelInt(positions, velocities, durations);
            }
            else
            {
                ROS_ERROR("Failed to load trajectory file");
            }
        }
    
    private:
        bool loadTrajectoryVel(const std::string& filename, 
                               std::vector<std::vector<double>>& positions,
                               std::vector<std::vector<double>>& velocities,
                               std::vector<double>& durations)
        {
            std::ifstream file(filename);
            if (!file.is_open()) {
                ROS_ERROR("Could not open file: %s", filename.c_str());
                return false;
            }
    
            std::string line;
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::vector<double> position_values;
                std::vector<double> velocity_values;
                std::string value;
    
                // Read 12 position values
                for (int i = 0; i < 12; i++) {
                    if (!std::getline(ss, value, ',')) {
                        ROS_ERROR("Invalid line in trajectory file");
                        return false;
                    }
                    position_values.push_back(std::stod(value));
                }
    
                // Read duration
                if (!std::getline(ss, value, ',')) {
                    ROS_ERROR("Invalid line in trajectory file");
                    return false;
                }
                double duration = std::stod(value);
    
                // Read 12 velocity values
                for (int i = 0; i < 12; i++) {
                    if (!std::getline(ss, value, ',')) {
                        ROS_ERROR("Invalid line in trajectory file");
                        return false;
                    }
                    velocity_values.push_back(std::stod(value));
                }
    
                positions.push_back(position_values);
                velocities.push_back(velocity_values);
                durations.push_back(duration);
            }
    
            return true;
        }
    
        void moveAllPosVelInt(const std::vector<std::vector<double>>& trajectories,
                              const std::vector<std::vector<double>>& velocities,
                              const std::vector<double>& durations)
        {
            if (trajectories.empty() || trajectories[0].size() != 12 || 
                velocities.empty() || velocities[0].size() != 12 || 
                trajectories.size() != velocities.size() || 
                trajectories.size() != durations.size()) {
                ROS_ERROR("Invalid trajectory data");
                return;
            }
    
            for (size_t step = 0; step < trajectories.size(); step++) {
                const auto& targetPos = trajectories[step];
                const auto& targetVel = velocities[step];
                double duration = durations[step];
    
                double start_time = ros::Time::now().toSec();
                double end_time = start_time + duration;
    
                while (ros::Time::now().toSec() < end_time) {
                    double percent = (ros::Time::now().toSec() - start_time) / duration;
                    percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1
    
                    for (int j = 0; j < 12; j++) {
                        lowCmd.motorCmd[j].q = targetPos[j];
                        lowCmd.motorCmd[j].dq = targetVel[j];
                        lowCmd.motorCmd[j].Kp = 5;
                        lowCmd.motorCmd[j].Kd = 1;
                        lowCmd.motorCmd[j].tau = 0;
                    }
    
                    udp.SetSend(lowCmd);
                    udp.Send();
                    usleep(1000);  // Sleep for 1ms
                }
            }
        }
    
        Safety safe;
        UDP udp;
        LowCmd lowCmd = {0};
        LowState lowState = {0};
        int motiontime = 0;
        bool trajectoryStarted = false;
    };
    
    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "trajectory_control_node");
        ros::NodeHandle nh;
    
        TrajectoryControl controller;
        ros::Rate loop_rate(500);
    
        while (ros::ok())
        {
            controller.UDPRecv();
            controller.RobotControl();
            controller.UDPSend();
            ros::spinOnce();
            loop_rate.sleep();
        }
    
        return 0;
    }

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

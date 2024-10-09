# Marty-Gazebo
## ros-to-real

## legged sdk UDP

    #include "unitree_legged_sdk/unitree_legged_sdk.h"
    #include <math.h>
    #include <iostream>
    #include <vector>
    #include <fstream>
    #include <sstream>
    
    using namespace UNITREE_LEGGED_SDK;
    
    class JumpControl
    {
    public:
        JumpControl() : safe(LeggedType::Go1),
                        udp(LOWLEVEL, 8090, "192.168.123.10", 8007)
        {
            udp.InitCmdData(cmd);
        }
        void UDPRecv();
        void UDPSend();
        void RobotControl();
        void loadTrajectory();
        void stand();
        void moveAllPosition(double* targetPos, double duration);
        void moveAllPosVelInt();
    
        Safety safe;
        UDP udp;
        LowCmd cmd = {0};
        LowState state = {0};
        int motiontime = 0;
        float dt = 0.002; // 0.001~0.01
    
        std::vector<std::vector<double>> positions;
        std::vector<std::vector<double>> velocities;
        std::vector<double> durations;
        size_t trajectoryIndex = 0;
        double trajectoryStartTime = 0;
        bool isStanding = false;
        bool isJumping = false;
    };
    
    void JumpControl::UDPRecv()
    {
        udp.Recv();
    }
    
    void JumpControl::UDPSend()
    {
        udp.Send();
    }
    
    void JumpControl::loadTrajectory()
    {
        std::string filename = "/home/ubuntu/Downloads/Trajectories/dq/trajectoryHop11dq.csv";
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Could not open file: " << filename << std::endl;
            return;
        }
    
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::vector<double> position_values;
            std::vector<double> velocity_values;
            std::string value;
    
            for (int i = 0; i < 12; i++) {
                std::getline(ss, value, ',');
                position_values.push_back(-1*(std::stod(value)));
            }
    
            std::getline(ss, value, ',');
            double duration = std::stod(value);
    
            for (int i = 0; i < 12; i++) {
                std::getline(ss, value, ',');
                velocity_values.push_back(-1*(std::stod(value)));
            }
    
            positions.push_back(position_values);
            velocities.push_back(velocity_values);
            durations.push_back(duration);
        }
        std::cout << "Trajectory loaded successfully." << std::endl;
    }
    
    void JumpControl::stand()
    {
        double standingPos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                                  0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        moveAllPosition(standingPos, 2000);
    }
    
    void JumpControl::moveAllPosition(double* targetPos, double duration)
    {
        double startPos[12];
        for(int j=0; j<12; j++) {
            startPos[j] = state.motorState[j].q;
        }
        double startTime = motiontime * dt;
        double endTime = startTime + duration * 0.001; // convert to seconds
    
        while (motiontime * dt < endTime) {
            double percent = (motiontime * dt - startTime) / (endTime - startTime);
            percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1
    
            for(int j=0; j<12; j++) {
                cmd.motorCmd[j].q = startPos[j]*(1-percent) + targetPos[j]*percent;
                cmd.motorCmd[j].dq = 0;
                cmd.motorCmd[j].Kp = 5;
                cmd.motorCmd[j].Kd = 1;
                cmd.motorCmd[j].tau = 0;
            }
    
            udp.SetSend(cmd);
            udp.Send();
            udp.Recv();
            udp.GetRecv(state);
    
            motiontime++;
            usleep(1000);
        }
    }
    
    void JumpControl::moveAllPosVelInt()
    {
        if (trajectoryIndex >= positions.size()) {
            isJumping = false;
            return;
        }
    
        double currentTime = motiontime * dt - trajectoryStartTime;
        double duration = durations[trajectoryIndex];
    
        if (currentTime >= duration) {
            trajectoryIndex++;
            trajectoryStartTime = motiontime * dt;
            return;
        }
    
        double percent = currentTime / duration;
        percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1
    
        for (int j = 0; j < 12; j++) {
            double startPos = (trajectoryIndex == 0) ? state.motorState[j].q : positions[trajectoryIndex-1][j];
            double endPos = positions[trajectoryIndex][j];
            double startVel = (trajectoryIndex == 0) ? 0 : velocities[trajectoryIndex-1][j];
            double endVel = velocities[trajectoryIndex][j];
    
            cmd.motorCmd[j].q = startPos * (1 - percent) + endPos * percent;
            cmd.motorCmd[j].dq = startVel * (1 - percent) + endVel * percent;
            cmd.motorCmd[j].Kp = 100;
            cmd.motorCmd[j].Kd = 4;
            cmd.motorCmd[j].tau = 0;
        }
    }
    
    void JumpControl::RobotControl()
    {
        motiontime++;
        udp.GetRecv(state);
    
        // gravity compensation
        cmd.motorCmd[FR_0].tau = -0.65f;
        cmd.motorCmd[FL_0].tau = +0.65f;
        cmd.motorCmd[RR_0].tau = -0.65f;
        cmd.motorCmd[RL_0].tau = +0.65f;
    
        if (!isStanding) {
            stand();
            isStanding = true;
            std::cout << "Robot is now in standing position." << std::endl;
            std::cout << "Press Enter to start the jump..." << std::endl;
            std::cin.ignore();
            isJumping = true;
            trajectoryStartTime = motiontime * dt;
        } else if (isJumping) {
            moveAllPosVelInt();
        }
    
        if (motiontime > 10) {
            safe.PowerProtect(cmd, state, 1);
        }
    
        udp.SetSend(cmd);
    }
    
    int main(void)
    {
        std::cout << "Communication level is set to LOW-level." << std::endl
                  << "WARNING: Make sure the robot is hung up." << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
    
        JumpControl jumpControl;
        jumpControl.loadTrajectory();
    
        LoopFunc loop_control("control_loop", jumpControl.dt, boost::bind(&JumpControl::RobotControl, &jumpControl));
        LoopFunc loop_udpSend("udp_send", jumpControl.dt, 3, boost::bind(&JumpControl::UDPSend, &jumpControl));
        LoopFunc loop_udpRecv("udp_recv", jumpControl.dt, 3, boost::bind(&JumpControl::UDPRecv, &jumpControl));
    
        loop_udpSend.start();
        loop_udpRecv.start();
        loop_control.start();
    
        while (1)
        {
            sleep(10);
        };
    
        return 0;
    }

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
    #include <iostream>
    
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
    
            // Initial stand and wait for user input
            if (motiontime == 10)
            {
                stand();
                std::cout << "Robot is in standing position. Press Enter to start the trajectory motion..." << std::endl;
                std::cin.get();
                trajectoryStarted = true;
            }
    
            // Trajectory control
            if (trajectoryStarted && !trajectoryFinished)
            {
                startTrajectory();
            }
    
            // Send command to robot
            udp.SetSend(lowCmd);
        }
    
        void stand()
        {
            double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                              0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
            moveAllPosition(pos, 2000);
        }
    
        void moveAllPosition(double* targetPos, double duration)
        {
            double pos[12], lastPos[12], percent;
            for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
            for(int i=1; i<=duration; i++){
                if(!ros::ok()) break;
                percent = (double)i/duration;
                for(int j=0; j<12; j++){
                    pos[j] = lastPos[j]*(1-percent) + targetPos[j]*percent;
                    lowCmd.motorCmd[j].q = pos[j];
                    lowCmd.motorCmd[j].Kp = 5;
                    lowCmd.motorCmd[j].dq = 0;
                    lowCmd.motorCmd[j].Kd = 1;
                    lowCmd.motorCmd[j].tau = 0;
                }
                udp.SetSend(lowCmd);
                udp.Send();
                usleep(1000);
            }
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
                trajectoryFinished = true;
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
    
            double lastPos[12];
            for (int j = 0; j < 12; j++) {
                lastPos[j] = lowState.motorState[j].q;
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
                        // Interpolate position
                        lowCmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
                        
                        // Interpolate velocity
                        double startVel = (step > 0) ? velocities[step-1][j] : 0;
                        lowCmd.motorCmd[j].dq = startVel * (1 - percent) + targetVel[j] * percent;
    
                        // Set control parameters
                        lowCmd.motorCmd[j].Kp = 5;
                        lowCmd.motorCmd[j].Kd = 1;
                        lowCmd.motorCmd[j].tau = 0;
                    }
    
                    udp.SetSend(lowCmd);
                    udp.Send();
                    usleep(1000);  // Sleep for 1ms
                }
    
                // Update lastPos for the next step
                for (int j = 0; j < 12; j++) {
                    lastPos[j] = targetPos[j];
                }
            }
        }
    
        Safety safe;
        UDP udp;
        LowCmd lowCmd = {0};
        LowState lowState = {0};
        int motiontime = 0;
        bool trajectoryStarted = false;
        bool trajectoryFinished = false;
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
    
        void stand() {   
            double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                              0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
            moveAllPosition(pos, 2*1000);
        }
    
        void moveAllPosition(double* targetPos, double duration) {
            double pos[12], lastPos[12], percent;
            for(int j=0; j<12; j++) lastPos[j] = low_state.motorState[j].q;
            for(int i=1; i<=duration; i++){
                if(!ros::ok()) break;
                percent = (double)i/duration;
                for(int j=0; j<12; j++){
                    low_cmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
                }
                sendCommand();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
    
        bool loadTrajectoryVel(const std::string& filename, 
                               std::vector<std::vector<double>>& positions,
                               std::vector<std::vector<double>>& velocities,
                               std::vector<double>& durations) {
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
                    position_values.push_back(-1*(std::stod(value)));
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
                    velocity_values.push_back(-1*(std::stod(value)));
                }
    
                positions.push_back(position_values);
                velocities.push_back(velocity_values);
                durations.push_back(duration);
            }
    
            return true;
        }
    
        void moveAllPosVelInt(const std::vector<std::vector<double>>& trajectories,
                              const std::vector<std::vector<double>>& velocities,
                              const std::vector<double>& durations) {
            if (trajectories.empty() || trajectories[0].size() != 12 || 
                velocities.empty() || velocities[0].size() != 12 || 
                trajectories.size() != velocities.size() || 
                trajectories.size() != durations.size()) {
                ROS_ERROR("Invalid trajectory data");
                return;
            }
    
            double lastPos[12];
            for (int j = 0; j < 12; j++) {
                lastPos[j] = low_state.motorState[j].q;
            }
    
            for (size_t step = 0; step < trajectories.size(); step++) {
                const auto& targetPos = trajectories[step];
                const auto& targetVel = velocities[step];
                double duration = durations[step];
    
                double start_time = ros::Time::now().toSec();
                double end_time = start_time + duration;
    
                while (ros::Time::now().toSec() < end_time) {
                    if (!ros::ok()) return;
    
                    double percent = (ros::Time::now().toSec() - start_time) / duration;
                    percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1
    
                    for (int j = 0; j < 12; j++) {
                        // Interpolate position
                        low_cmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
                        
                        // Interpolate velocity
                        double startVel = (step > 0) ? velocities[step-1][j] : 0;
                        low_cmd.motorCmd[j].dq = startVel * (1 - percent) + targetVel[j] * percent;
    
                        // Set control parameters
                        low_cmd.motorCmd[j].Kp = 100;  // Adjust as needed
                        low_cmd.motorCmd[j].Kd = 4;  // Adjust as needed
                        low_cmd.motorCmd[j].tau = 0; // Set to 0 for position/velocity control
                    }
    
                    sendCommand();
                    ros::spinOnce();
                    loop_rate.sleep();
                }
    
                // Update lastPos for the next step
                for (int j = 0; j < 12; j++) {
                    lastPos[j] = targetPos[j];
                }
            }
            for (int j = 0; j < 12; j++) {
                low_cmd.motorCmd[j].dq = 0;
            }
        }
    
        void run() {
            std::cout << "Making the robot stand. Please wait..." << std::endl;
            stand();
            
            std::vector<std::vector<double>> positions, velocities;
            std::vector<double> durations;
            if (!loadTrajectoryVel("/home/ubuntu/Downloads/Trajectories/dq/trajectoryHop11dq.csv", positions, velocities, durations)) {
                return;
            }
    
            std::cout << "Trajectory loaded. Robot is in standing position." << std::endl;
            std::cout << "Press Enter to start the jump..." << std::endl;
            std::cin.ignore();
    
            moveAllPosVelInt(positions, velocities, durations);
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

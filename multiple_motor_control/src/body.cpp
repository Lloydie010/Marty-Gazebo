/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"
#include <thread>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

namespace unitree_model {

ros::Publisher servo_pub[12];
unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 8;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    }
}

void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2*1000);
}

void motion_init()
{
    paramInit();
    stand();
}

void sendServoCmd()
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    usleep(1000);
}
void move_groups(int group_number, double position, double duration)
{
    paramInit();

    double pos[12], lastPos[12], percent;
    for (int j = 0; j < 12; j++)
        lastPos[j] = lowState.motorState[j].q;

    std::vector<std::thread> threads;

    for (int i = 1; i <= duration; i++) {
        if (!ros::ok())
            break;

        percent = static_cast<double>(i) / duration;

        if (group_number == 0) {
            threads.emplace_back([&lastPos, &position, &percent]() {
                lowCmd.motorCmd[0].q = lastPos[0] * (1 - percent) + position * percent;
                lowCmd.motorCmd[3].q = lastPos[3] * (1 - percent) + position * percent;
                lowCmd.motorCmd[6].q = lastPos[6] * (1 - percent) + position * percent;
                lowCmd.motorCmd[9].q = lastPos[9] * (1 - percent) + position * percent;
            });
        } else if (group_number == 1) {
            threads.emplace_back([&lastPos, &position, &percent]() {
                lowCmd.motorCmd[1].q = lastPos[1] * (1 - percent) + position * percent;
                lowCmd.motorCmd[4].q = lastPos[4] * (1 - percent) + position * percent;
                lowCmd.motorCmd[7].q = lastPos[7] * (1 - percent) + position * percent;
                lowCmd.motorCmd[10].q = lastPos[10] * (1 - percent) + position * percent;
            });
        } else if (group_number == 2) {
            threads.emplace_back([&lastPos, &position, &percent]() {
                lowCmd.motorCmd[2].q = lastPos[2] * (1 - percent) + position * percent;
                lowCmd.motorCmd[5].q = lastPos[5] * (1 - percent) + position * percent;
                lowCmd.motorCmd[8].q = lastPos[8] * (1 - percent) + position * percent;
                lowCmd.motorCmd[11].q = lastPos[11] * (1 - percent) + position * percent;
            });
        }

        sendServoCmd();
    }

    // Wait for all threads to finish
    for (auto& thread : threads) {
        thread.join();
    }
}

void move_groups1(int group_number,double position, double duration)
{   
    paramInit();

    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
     for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        //group 0 will have the following items 0
                                           //   3
                                           //   6
                                           //   9
 
        if(group_number==0){
            lowCmd.motorCmd[0].q = lastPos[0]*(1-percent) + position*percent;
            lowCmd.motorCmd[3].q = lastPos[3]*(1-percent) + position*percent;
            lowCmd.motorCmd[6].q = lastPos[6]*(1-percent) + position*percent;
            lowCmd.motorCmd[9].q = lastPos[9]*(1-percent) + position*percent;                
            
        }
        if(group_number==1){
            lowCmd.motorCmd[1].q = lastPos[1]*(1-percent) + position*percent;
            lowCmd.motorCmd[4].q = lastPos[4]*(1-percent) + position*percent;
            lowCmd.motorCmd[7].q = lastPos[7]*(1-percent) + position*percent;
            lowCmd.motorCmd[10].q = lastPos[10]*(1-percent) + position*percent; 
        }
        if(group_number==2){
            lowCmd.motorCmd[2].q = lastPos[2]*(1-percent) + position*percent;
            lowCmd.motorCmd[5].q = lastPos[5]*(1-percent) + position*percent;
            lowCmd.motorCmd[8].q = lastPos[8]*(1-percent) + position*percent;
            lowCmd.motorCmd[11].q = lastPos[11]*(1-percent) + position*percent; 
        }
        
        sendServoCmd();
    }
}

void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
    }
}

bool loadTrajectoryFromFile(const std::string& filename, std::vector<std::vector<double>>& trajectories, std::vector<double>& durations)
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

        trajectories.push_back(position_values);
        durations.push_back(duration);
    }

    return true;
}

void moveAllPositionMult(const std::vector<std::vector<double>>& trajectories, const std::vector<double>& durations)
{
    if (trajectories.empty() || trajectories[0].size() != 12 || trajectories.size() != durations.size()) {
        ROS_ERROR("Invalid trajectory data");
        return;
    }

    //paramInit();

    double lastPos[12];
    for (int j = 0; j < 12; j++) {
        lastPos[j] = lowState.motorState[j].q;
    }

    for (size_t step = 0; step < trajectories.size(); step++) {
        const auto& targetPos = trajectories[step];
        double duration = durations[step];

        double start_time = ros::Time::now().toSec();
        double end_time = start_time + duration;

        while (ros::Time::now().toSec() < end_time) {
            if (!ros::ok()) return;

            double percent = (ros::Time::now().toSec() - start_time) / duration;
            percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1

            for (int j = 0; j < 12; j++) {
                lowCmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
                
                lowCmd.motorCmd[j].Kp = 70;  // Adjust as needed
                lowCmd.motorCmd[j].Kd = 2;  // Adjust as needed
                //lowCmd.motorCmd[j].tau = 0; // Set to 0 for position/velocity control
            }

            sendServoCmd();
            ros::spinOnce();
        }

        // Update lastPos for the next step
        for (int j = 0; j < 12; j++) {
            lastPos[j] = lowCmd.motorCmd[j].q;
        }
    }
}

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
            if (!ros::ok()) return;

            double percent = (ros::Time::now().toSec() - start_time) / duration;
            percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1

            for (int j = 0; j < 12; j++) {
                // Interpolate position
                lowCmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
                
                // Interpolate velocity
                double startVel = (step > 0) ? velocities[step-1][j] : 0;
                lowCmd.motorCmd[j].dq = startVel * (1 - percent) + targetVel[j] * percent;

                // Set control parameters
                lowCmd.motorCmd[j].Kp = 100;  // Adjust as needed
                lowCmd.motorCmd[j].Kd = 4;  // Adjust as needed
                lowCmd.motorCmd[j].tau = 0; // Set to 0 for position/velocity control
            }

            sendServoCmd();
            ros::spinOnce();
        }


        // Update lastPos for the next step
        for (int j = 0; j < 12; j++) {
            lastPos[j] = targetPos[j];
        }
    }
    for (int j = 0; j < 12; j++) {
            lowCmd.motorCmd[j].dq = 0;
    }
}

void moveAllPosVel(const std::vector<std::vector<double>>& trajectories,
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

    //paramInit();

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
            if (!ros::ok()) return;

            double percent = (ros::Time::now().toSec() - start_time) / duration;
            percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1

            for (int j = 0; j < 12; j++) {
                lowCmd.motorCmd[j].q = lastPos[j] * (1 - percent) + targetPos[j] * percent;
                lowCmd.motorCmd[j].dq = targetVel[j];
                lowCmd.motorCmd[j].Kp = 70;  // Set to 0 for velocity control
                lowCmd.motorCmd[j].Kd = 4;  // Adjust as needed for your robot
                lowCmd.motorCmd[j].tau = 0;
            }

            sendServoCmd();
            ros::spinOnce();
        }


        // Update lastPos for the next step
        for (int j = 0; j < 12; j++) {
            lastPos[j] = lowCmd.motorCmd[j].q;
        }
    }
    for (int j = 0; j < 12; j++) {
            lowCmd.motorCmd[j].dq = 0;
    }
}

void moveAllTorque(const std::vector<std::vector<double>>& torques,
                                   const std::vector<double>& durations)
{
    if (torques.empty() || torques[0].size() != 12 || torques.size() != durations.size()) {
        ROS_ERROR("Invalid torque data");
        return;
    }

    //paramInit();

    std::vector<double> lastTorque(12, 0.0);  // Initialize with zeros

    for (size_t step = 0; step < torques.size(); step++) {
        const auto& targetTorque = torques[step];
        double duration = durations[step];

        double start_time = ros::Time::now().toSec();
        double end_time = start_time + duration;

        while (ros::Time::now().toSec() < end_time) {
            if (!ros::ok()) return;

            double percent = (ros::Time::now().toSec() - start_time) / duration;
            percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1

            for (int j = 0; j < 12; j++) {
                // Interpolate torque
                double interpolatedTorque = lastTorque[j] * (1 - percent) + targetTorque[j] * percent;

                lowCmd.motorCmd[j].mode = 0x0A;  // Torque control mode
                lowCmd.motorCmd[j].tau = interpolatedTorque;
                lowCmd.motorCmd[j].Kp = 0;
                lowCmd.motorCmd[j].Kd = 0;
                lowCmd.motorCmd[j].q = 0;   // Not used in torque control
                lowCmd.motorCmd[j].dq = 0;  // Not used in torque control
            }

            sendServoCmd();
            ros::spinOnce();
        }

        // Update lastTorque for the next step
        lastTorque = targetTorque;
    }
}

}

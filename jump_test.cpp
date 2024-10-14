#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <stdexcept>

using namespace UNITREE_LEGGED_SDK;

class JumpControl
{
public:
    JumpControl(): 
        safe(LeggedType::Go1),
        udp(LOWLEVEL, 8090, "192.168.123.10", 8007) {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void loadTrajectory();
    void moveAllPosition(double* targetPos, double duration);
    void moveAllPosVelInt(double initDuration);
    void writeToCSV(const std::string& filename, const std::vector<std::vector<float>>& data);

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motionTime = 0;
    float dt = 0.002; // 0.001~0.01

    std::vector<std::vector<double>> positions;
    std::vector<std::vector<double>> velocities;
    std::vector<double> durations;
    std::vector<std::vector<float>> data; //FR, FL positions, velocities, imu angle, accelleration, feet position
    double controlPos[12];
    int startTime = 0;
    int trajectoryIndex = 0;
    int prevIndex = -1;
    double motionStartTime = 0;
    bool motionStarted = false;
    bool isStanding = false;
    bool isJumping = false;
    bool init = false;
    bool controllable = false;
    bool isLowered = false;
    bool isTrajectory = false;
    bool isLanding = false;
};

void JumpControl::UDPRecv()
{
    udp.Recv();
}

void JumpControl::UDPSend()
{
    udp.Send();
}

void JumpControl::writeToCSV(const std::string& filename, const std::vector<std::vector<float>>& data) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file");
    }
    
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }
    
    file.close();
}

void JumpControl::loadTrajectory()
{
    std::string filename = "/home/ubuntu/Downloads/Trajectories/dq/trajectoryHop20dq.csv";
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



void JumpControl::moveAllPosition(double* targetPos, double duration)
{
    cout << motionStarted << endl;
    if (!motionStarted) {
        motionStarted = true;
        controllable = false;
        // std::cout << "Press Enter to start to stand..." << std::endl;
        // std::cin.ignore();
        startTime = motionTime;
        // cout << motionStarted << endl;
    }
    udp.GetRecv(state);
    double startPos[12];
    for(int j=0; j<12; j++) {
        startPos[j] = state.motorState[j].q;
    }
    double endTime = startTime + duration*500; // convert to seconds
    double percent = (motionTime - startTime)/(duration*500);
    percent = std::min(std::max(percent, 0.0), 1.0);
    // cout << percent << endl;
    if (startPos[2] == 0) {
        cout << "Skipped" << endl;
    } else {

        // cout << percent << endl;
        // while (motionTime * dt < endTime) {
        //     double percent = (motionTime * dt - startTime) / (endTime - startTime);
        //     percent = std::min(1.0, std::max(0.0, percent));  // Clamp between 0 and 1
        // cout << "STATE" << endl;
        for(int j=0; j<12; j++) {
            cmd.motorCmd[j].q = startPos[j]*(1-percent) + targetPos[j]*(percent);
            cmd.motorCmd[j].dq = 0;
            cmd.motorCmd[j].Kp = 100;
            cmd.motorCmd[j].Kd = 4;
            // cmd.motorCmd[j].tau = 0;
            // cout << j << endl;
            // cout << startPos[j] << endl;
        }
    }
    // sleep(2);
    if (motionTime >= endTime) {
        motionStarted = false;
        controllable = true;
        cout << "Finished" << endl;
    }

    udp.SetSend(cmd);
    // udp.Send();
    // udp.Recv();
    // udp.GetRecv(state);

    // }
}

void JumpControl::moveAllPosVelInt(double initDuration)
{
    durations[0] = initDuration;
    // cout << "move all" << endl;
    if (trajectoryIndex > prevIndex) {
        motionStarted = true;
        controllable = false;
        startTime = motionTime;
        prevIndex++;
        cout << "NEXT TRAJECTORY STARTING" << endl;
        cout << trajectoryIndex << endl;
        // cout << motionStarted << endl;
    }

    udp.GetRecv(state);
    double startPos[12];
    for(int j=0; j<12; j++) {
        startPos[j] = state.motorState[j].q;
    }
    double endTime = startTime + durations[trajectoryIndex]*500; // convert to seconds
    double percent = (motionTime - startTime)/(durations[trajectoryIndex]*500);
    percent = std::min(std::max(percent, 0.0), 1.0);
    // cout << percent << endl;
    if (0) {
        // cout << "Skipped" << endl;
    } else {

        for (int j = 0; j < 12; j++) {
            double startPos = (trajectoryIndex == 0) ? state.motorState[j].q : positions[trajectoryIndex-1][j];
            double endPos = positions[trajectoryIndex][j];
            double startVel = (trajectoryIndex == 0) ? 0 : velocities[trajectoryIndex-1][j];
            double endVel = velocities[trajectoryIndex][j];
            cout << "MOTOR:" << j << endl;
            cout << "POS:" << (startPos * (1 - percent) + endPos * percent) << endl;
            cout << "VEL:" << (startVel * (1 - percent) + endVel * percent) << endl;
            cmd.motorCmd[j].q = startPos * (1 - percent) + endPos * percent;
            // cmd.motorCmd[j].dq = startVel * (1 - percent) + endVel * percent;
            // cmd.motorCmd[j].dq = 0;
            cmd.motorCmd[j].Kp = 100;
            cmd.motorCmd[j].Kd = 4;
            // cmd.motorCmd[j].tau = 0;
        }

    }
    if (motionTime >= endTime) {

        std::vector<float> row = {state.motorState[1].q, state.motorState[2].q, state.motorState[7].q, state.motorState[8].q, 
                                state.motorState[1].dq, state.motorState[2].dq, state.motorState[7].dq, state.motorState[8].dq, 
                                state.motorState[1].tauEst, state.motorState[2].tauEst,state.motorState[7].tauEst, state.motorState[8].tauEst, 
                                state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2], 
                                state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3], 
                                state.imu.rpy[0], (float)state.footForce[0], (float)state.footForce[2], (float)durations[trajectoryIndex]};
        data.push_back(row);

        trajectoryIndex++;
        cout << "NEXT TRAJECTORY" << endl;
        if (trajectoryIndex == positions.size())
        {
            cout << "TRAJECTORY COMPLETE" << endl;
            isTrajectory = true;
            // for (int j = 0; j < 12; j++) {
            //     cmd.motorCmd[j].dq = 0;
            // }
            writeToCSV("/home/ubuntu/Documents/SensorDataGo1/Hop20gnh.csv", data);
        }
    }
}

void JumpControl::RobotControl()
{
    if (!init) {
        for (int i = 0; i < 12; i++)
        {
            cmd.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
            cmd.motorCmd[i].q = PosStopF; // 禁止位置环
            cmd.motorCmd[i].Kp = 0;
            cmd.motorCmd[i].dq = VelStopF; // 禁止速度环
            cmd.motorCmd[i].Kd = 0;
            cmd.motorCmd[i].tau = 0;
        }
        init = true;
    }

    motionTime++;
   
    // std::cout << motionTime << std::endl;

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    // std::cout << "Shoulders" << std::endl;
    
    // cmd.motorCmd[FR_0].q = 0;
    // cmd.motorCmd[FL_0].q = 0;
    // cmd.motorCmd[RR_0].q = 0;
    // cmd.motorCmd[RL_0].q = 0;

    udp.SetSend(cmd);

    udp.GetRecv(state);

    if (motionTime > 200) {

        // isTrajectory = true;

        isLowered = true;
        isJumping = true;
        isLanding = true;
        if (!isStanding) {
            // double pos[12] = {1.0, 1.67, -2.64, 1.0, 1.67, -2.64, 
            //                     1.0, 1.67, -2.64, 1.0, 1.67, -2.64};
            double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
            moveAllPosition(pos, 2);
            if (!motionStarted)
            {
                cout << "End Standing" << endl;
                isStanding = true;
            }
        } else if (!isTrajectory) {
            if (!motionStarted)
            {
                std::cout << "Robot is now in standing position." << std::endl;
                std::cout << "Press Enter to start the jump..." << std::endl;
                std::cin.ignore();
            }
            moveAllPosVelInt(0.3);
        } else if (!isLowered) {
            if (!motionStarted)
            {
                std::cout << "Robot is now in standing position." << std::endl;
                std::cout << "Press Enter to start the jump..." << std::endl;
                std::cin.ignore();
            }
            double pos[12] = {0.0, 0.87, -1.9, -0.0, 0.87, -1.9, 
                            0.0, 0.87, -1.9, -0.0, 0.87, -1.9};
            moveAllPosition(pos, 2);
            if (!motionStarted)
            {
                cout << "End lowering" << endl;
                isLowered = true;
            }

        } else if (!isJumping) {
            double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                            0.0, 0.67, -1.1, -0.0, 0.67, -1.1};
            moveAllPosition(pos, 0.04);
            if (motionStarted == false)
            {
                cout << "End Jumping" << endl;
                isJumping = true;
            }

        } else if (!isLanding) {
            double pos[12] = {0.0, 0.87, -1.7, -0.0, 0.87, -1.7, 
                            0.0, 0.87, -1.7, -0.0, 0.87, -1.7};
            moveAllPosition(pos, 0.04);
            if (!motionStarted)
            {
                cout << "End Jumping" << endl;
                isLanding = true;
            }

        } else {
            // cout << "done" << endl;
        }
    }
    
    // if (controllable) {
        
    //     for(int j=0; j<12; j++) {
    //         // cmd.motorCmd[j].q = controlPos[j];
    //         // cmd.motorCmd[j].dq = 0;
    //         // cmd.motorCmd[j].Kp = 5;
    //         // cmd.motorCmd[j].Kd = 1;
    //         cout << controlPos[j] << endl;
    //         sleep(0.2);
    //     }
    // }



    if (motionTime > 10) {
        safe.PositionLimit(cmd);
        int res1 = safe.PowerProtect(cmd, state, 7);
        // You can uncomment it for position protection
        // int res2 = safe.PositionProtect(cmd, state, 10);
        // cout << res1 << endl;
        if(res1 < 0) exit(-1);
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
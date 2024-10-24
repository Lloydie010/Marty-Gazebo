/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState;

void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);
bool loadTrajectoryFromFile(const std::string& filename, std::vector<std::vector<double>>& trajectories, std::vector<double>& durations);
void moveAllPositionMult(const std::vector<std::vector<double>>& trajectories, const std::vector<double>& durations);
bool loadTrajectoryVel(const std::string& filename, 
                            std::vector<std::vector<double>>& positions,
                            std::vector<std::vector<double>>& velocities,
                            std::vector<double>& durations);
void moveAllPosVelInt(const std::vector<std::vector<double>>& trajectories,
                     const std::vector<std::vector<double>>& velocities,
                     const std::vector<double>& durations);
void moveAllPosVel(const std::vector<std::vector<double>>& trajectories,
                     const std::vector<std::vector<double>>& velocities,
                     const std::vector<double>& durations);
void moveAllTorque(const std::vector<std::vector<double>>& torques, const std::vector<double>& durations);
}

#endif

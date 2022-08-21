#ifndef TASK2_H
#define TASK2_H

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include "sensor_msgs/JointState.h"

// KDL
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// Reflexxes
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>


class TASK2 : public ros::NodeHandle{
    private:

        ros::NodeHandle nh;


        ros::Publisher joint_trajectory_pub_;

        int ResultValue_ = 0;
        ReflexxesAPI* RML_;
        RMLPositionInputParameters* IP_;
        RMLPositionOutputParameters* OP_;
        RMLPositionFlags Flags_;
        trajectory_msgs::JointTrajectory traj_;

        // KDL::Tree tree;
        // KDL::Chain chain;
        // KDL::ChainFkSolverPos* fk_pos;
        // KDL::ChainIkSolverPos* ik_pos;
        // KDL::ChainIkSolverVel* ik_vel;

        // KDL::JntArray joint_state;
        void init_reflexxes_and_traj(std::vector<double> initial_Joints_positions);
        void get_ur5_states(const sensor_msgs::JointState& states_msg);
        void update_reflexxes_parameters(std::vector<double> initial_Joints_positions,std::vector<double>  initial_Joints_velocities,std::vector<double>  initial_Joints_accelerations, std::vector<double> target_Joints_positions);
        void update_reflexxes_parameters(std::vector<double> target_Joints_positions);
        bool go_to_pose(int k);
    public:
        TASK2(ros::NodeHandle& nh);
        ~TASK2();
};

#endif // TASK2_H
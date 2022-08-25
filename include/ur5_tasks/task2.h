#ifndef TASK2_H
#define TASK2_H

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
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
#include <urdf/model.h>

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
        std::vector<double> initial_Joints_positions_;
        std::vector<double> initial_Joints_velocities_;
        std::vector<double> initial_Joints_accelerations_;
        std::vector<double> target_Joints_positions_;
        void init_reflexxes_and_traj();
        void get_ur5_states(const sensor_msgs::JointState& states_msg);
        void update_reflexxes_parameters();
        bool go_to_pose(int k);
        void load_parameters();
        void inverse_kinematics(std::string robot_description, std::vector<double> pose1, std::vector<double> pose2, double linear_velocity, double linear_accelaration);
        void set_init_goal_positions(std::vector<double> initial_Joints_positions, std::vector<double> initial_Joints_velocities, std::vector<double> initial_Joints_acceleration ,std::vector<double> target_Joints_positions);
    public:
        TASK2(ros::NodeHandle& nh);
        ~TASK2();
};

#endif // TASK2_H
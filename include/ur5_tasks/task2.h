#ifndef TASK2_H
#define TASK2_H

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>

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


class TASK2 {
    private:
        ros::Publisher joint_trajectory_pub_;
        trajectory_msgs::JointTrajectory traj_;

        std::vector<double> initial_Joints_positions_;
        std::vector<double> initial_Joints_velocities_;
        std::vector<double> initial_Joints_accelerations_;
        std::vector<double> target_Joints_positions_;

        ReflexxesAPI* RML_;
        RMLPositionInputParameters* IP_;
        RMLPositionOutputParameters* OP_;

        KDL::JntArray target_pose_joints_;

        void load_parameters(ros::NodeHandle& nh);
        void inverse_kinematics(std::string robot_description, std::vector<double> pose1, std::vector<double> pose2, double linear_velocity, double linear_accelaration);
        void init_trajectory();
        void init_reflexxes();
        void update_reflexxes_parameters();
        bool go_to_target();

    public:
        TASK2(ros::NodeHandle& nh);
        ~TASK2();
};

#endif // TASK2_H
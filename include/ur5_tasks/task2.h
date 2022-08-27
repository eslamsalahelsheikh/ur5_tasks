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

/**
 * @brief This class is used to control the UR5 joints using the Reflexxes Motion Library & KDL inverse kinamatics solver.
 */
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
        /**
         * @brief this function loads all the parameters from the parameter server based on the motion type.
         * @param nh node handle
         */
        void load_parameters(ros::NodeHandle& nh);

        /**
         * @brief this function uses KDL to calculate the inverse kinematics of the UR5 joints.
         * @param robot_description robot description from the parameter server
         * @param pos1 initial position of the UR5 joints in cartesian space
         * @param pos2 target position of the UR5 joints in cartesian space
         * @param linear_velocity linear velocity of the UR5 joints in cartesian space
         * @param linear_acceleration linear acceleration of the UR5 joints in cartesian space
         * @
         */
        void inverse_kinematics(std::string robot_description, std::vector<double> pose1, std::vector<double> pose2, double linear_velocity, double linear_accelaration);

        /**
         * @brief this function initialize the trajectory publisher with the names and initial positions of the UR5 joints.
         */
        void init_trajectory();

        /**
         * @brief this function initialize the Reflexxes Motion Library and the RMLPositionInputParameters.
        */
        void init_reflexxes();

        /**
         * @brief this function updates the Reflexxes parameters using the initial and target positions of the UR5 joints.
         */
        void update_reflexxes_parameters();

        /**
         * @brief this function calculate the next step of the trajectory in joint space and publish it using the trajectory publisher.
         * @return true when the trajectory is finished and goal is reached.
         * @return false when the goal is not reached yet.
         */
        bool go_to_target();

    public:
        /**
         * @brief constructor of the class.
         * @param nh node handle
         */
        TASK2(ros::NodeHandle& nh);
        /**
         * @brief destructor of the class.
         */
        ~TASK2();
};

#endif // TASK2_H
// #include "std_msgs/String.h"
#include "ur5_tasks/task2.h"



#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "kdl/chainiksolver.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_nr.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_conversions/kdl_msg.h"
#include <ros/package.h>
#include <moveit/rdf_loader/rdf_loader.h>

geometry_msgs::PoseStamped pippo_;
KDL::JntArray   q_init(6);
unsigned int k=0;
KDL::Frame desired_frame;


void get_ur5_states(const sensor_msgs::JointState& pluto){
    for(int i=0;i<6;i++){
    q_init.data[i]=pluto.position[i];
    }
}
int main(int argc,char **argv){
    ros::init(argc,argv,"joint_node");
    ros::NodeHandle nh_;
    ros::Subscriber sub_ur5_joint_ =nh_.subscribe("joint_states",1,get_ur5_states);
    ros::Publisher joint_trajectory_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);
    ros::Rate loop_rate(1);
    trajectory_msgs::JointTrajectory traj;

    // building KDL tree
    KDL::Tree my_tree;
   	std::string urdf_path = ros::package::getPath("ur5_tasks");
	if(urdf_path.empty()) ROS_ERROR("ur_description package path was not found");
	urdf_path += "/urdf/ur5.urdf";

    if (!kdl_parser::treeFromFile(urdf_path, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    KDL::Chain chain;
    my_tree.getChain("world","tool0",chain);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    KDL::ChainIkSolverPos_NR ik_solver(chain, fk_solver, ik_solver_vel, 1000, 100);

    KDL::JntArray q_out(chain.getNrOfJoints());

    // set the desired goal position in cartesian space
    desired_frame.p.x(0.5);
    desired_frame.p.y(0.5);
    desired_frame.p.z(0.5);
    ik_solver.CartToJnt(q_init,desired_frame,q_out);

    traj.header.stamp = ros::Time::now();
    traj.joint_names.resize(6);
    traj.joint_names[0] = "shoulder_pan_joint";
    traj.joint_names[1] = "shoulder_lift_joint";
    traj.joint_names[2] = "elbow_joint";
    traj.joint_names[3] = "wrist_1_joint";
    traj.joint_names[4] = "wrist_2_joint";
    traj.joint_names[5] = "wrist_3_joint";
    traj.points.resize(1);
    traj.points[0].positions.resize(6);
    traj.points[0].positions[0]=(q_out.data(0));
    traj.points[0].positions[1]=(q_out.data(1));
    traj.points[0].positions[2]=(q_out.data(2));
    traj.points[0].positions[3]=(q_out.data(3));
    traj.points[0].positions[4]=(q_out.data(4));
    traj.points[0].positions[5]=(q_out.data(5));  


    traj.points[0].time_from_start = ros::Duration(1.0);
    joint_trajectory_pub.publish(traj);


  while (ros::ok()){
    ik_solver.CartToJnt(q_init,desired_frame,q_out);
    // std::cout<<q_out.data<<std::endl;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.resize(6);
    traj.points.resize(k+1);
    traj.points[k].positions.resize(6);
    traj.points[k].positions[0]=(q_out.data(0));
    traj.points[k].positions[1]=(q_out.data(1));
    traj.points[k].positions[2]=(q_out.data(2));
    traj.points[k].positions[3]=(q_out.data(3));
    traj.points[k].positions[4]=(q_out.data(4));
    traj.points[k].positions[5]=(q_out.data(5));  

    traj.points[k].time_from_start = ros::Duration((k+1)*1.0);
    joint_trajectory_pub.publish(traj);
    k++;
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}
// used example in: http://www.reflexxes.ws/software/typeiirml/v1.2.6/docs/page__code_01__r_m_l_position_sample_application.html
#include "ur5_tasks/task2.h"

#define CYCLE_TIME_IN_SECONDS 0.1
#define DOF 6


TASK2::TASK2(ros::NodeHandle& nh) {
    ros::Rate loop_rate(10);
    joint_trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);
    load_parameters(nh);
    init_trajectory();
    init_reflexxes();

    // Starting the control loop
    bool goal_reached = false;
    auto start_time = std::chrono::system_clock::now();
    while (ros::ok()){
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start_time;
        if (elapsed_seconds.count() > 3.0 and !goal_reached)    // wait for 3 seconds until the robot reaches the initial pose
        {
            goal_reached = go_to_target();
        }
        else if (goal_reached) {
            std::cout << "Goal reached" << std::endl;
            exit(0);  // kill the node after the goal is reached
        }
        else {
            joint_trajectory_pub_.publish(traj_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

};


TASK2::~TASK2()
{
    delete RML_;
    delete IP_;
    delete OP_;
}

void TASK2::load_parameters(ros::NodeHandle& nh){

    int motion_number = 1;
    nh.getParam("motion_type", motion_number);
    initial_Joints_positions_.resize(DOF);
    initial_Joints_velocities_.resize(DOF);
    initial_Joints_accelerations_.resize(DOF);
    target_Joints_positions_.resize(DOF);

    switch (motion_number) {
        case 1: // motion 1 (joint space motion)
        {
            nh.getParam("first_motion/joint1_conf/shoulder_pan_joint", initial_Joints_positions_[0]);
            nh.getParam("first_motion/joint1_conf/shoulder_lift_joint", initial_Joints_positions_[1]);
            nh.getParam("first_motion/joint1_conf/elbow_joint", initial_Joints_positions_[2]);
            nh.getParam("first_motion/joint1_conf/wrist_1_joint", initial_Joints_positions_[3]);
            nh.getParam("first_motion/joint1_conf/wrist_2_joint", initial_Joints_positions_[4]);
            nh.getParam("first_motion/joint1_conf/wrist_3_joint", initial_Joints_positions_[5]);

            nh.getParam("first_motion/joints_velocity/shoulder_pan_joint", initial_Joints_velocities_[0]);
            nh.getParam("first_motion/joints_velocity/shoulder_lift_joint", initial_Joints_velocities_[1]);
            nh.getParam("first_motion/joints_velocity/elbow_joint", initial_Joints_velocities_[2]);
            nh.getParam("first_motion/joints_velocity/wrist_1_joint", initial_Joints_velocities_[3]);
            nh.getParam("first_motion/joints_velocity/wrist_2_joint", initial_Joints_velocities_[4]);
            nh.getParam("first_motion/joints_velocity/wrist_3_joint", initial_Joints_velocities_[5]);

            nh.getParam("first_motion/joints_acceleration/shoulder_pan_joint", initial_Joints_accelerations_[0]);
            nh.getParam("first_motion/joints_acceleration/shoulder_lift_joint", initial_Joints_accelerations_[1]);
            nh.getParam("first_motion/joints_acceleration/elbow_joint", initial_Joints_accelerations_[2]);
            nh.getParam("first_motion/joints_acceleration/wrist_1_joint", initial_Joints_accelerations_[3]);
            nh.getParam("first_motion/joints_acceleration/wrist_2_joint", initial_Joints_accelerations_[4]);
            nh.getParam("first_motion/joints_acceleration/wrist_3_joint", initial_Joints_accelerations_[5]);

            nh.getParam("first_motion/joint2_conf/shoulder_pan_joint", target_Joints_positions_[0]);
            nh.getParam("first_motion/joint2_conf/shoulder_lift_joint", target_Joints_positions_[1]);
            nh.getParam("first_motion/joint2_conf/elbow_joint", target_Joints_positions_[2]);
            nh.getParam("first_motion/joint2_conf/wrist_1_joint", target_Joints_positions_[3]);
            nh.getParam("first_motion/joint2_conf/wrist_2_joint", target_Joints_positions_[4]);
            nh.getParam("first_motion/joint2_conf/wrist_3_joint", target_Joints_positions_[5]);
            break;
        }
        case 2: // motion 2 (cartesian motion)
        {
            std::string robot_description;
            nh.getParam("robot_description", robot_description);
            std::vector<double> pose1(3);
            std::vector<double> pose2(3);
            double linear_velocity;
            double linear_accelaration;
            nh.getParam("second_motion/pose1/x", pose1[0]);
            nh.getParam("second_motion/pose1/y", pose1[1]);
            nh.getParam("second_motion/pose1/z", pose1[2]);

            nh.getParam("second_motion/pose2/x", pose2[0]);
            nh.getParam("second_motion/pose2/y", pose2[1]);
            nh.getParam("second_motion/pose2/z", pose2[2]);

            nh.getParam("second_motion/linear_velocity", linear_velocity);
            nh.getParam("second_motion/linear_acceleration", linear_accelaration);

            // Apply a transformation to the pose1 and pose2 from cartesian to joint space
            inverse_kinematics(robot_description, pose1, pose2, linear_velocity, linear_accelaration);
            break;
        }
    }
}
void TASK2::inverse_kinematics(std::string robot_description, std::vector<double> pose1, std::vector<double> pose2, double linear_velocity, double linear_accelaration){
    // initiailize the KDL Tree
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(robot_description, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    KDL::Chain chain;
    my_tree.getChain("world","tool0",chain);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    KDL::ChainIkSolverPos_NR ik_solver(chain, fk_solver, ik_solver_vel, 1000, 100);

    KDL::JntArray q_init(DOF),initial_pose_joints(DOF),target_pose_joints(DOF);
    KDL::Frame initial_pose_cartesian,taret_pose_cartesian;

    // set the initial and desired goal position in cartesian space
    initial_pose_cartesian.p.x(pose1[0]);
    initial_pose_cartesian.p.y(pose1[1]);
    initial_pose_cartesian.p.z(pose1[2]);
    taret_pose_cartesian.p.x(pose2[0]);
    taret_pose_cartesian.p.y(pose2[1]);
    taret_pose_cartesian.p.z(pose2[2]);

    // calculate the initial and target position in joint space
    ik_solver.CartToJnt(q_init,initial_pose_cartesian,initial_pose_joints);
    ik_solver.CartToJnt(q_init,taret_pose_cartesian,target_pose_joints);

    for (int i = 0; i < DOF; i++){
        initial_Joints_positions_[i] = initial_pose_joints(i);
        initial_Joints_velocities_[i] = linear_velocity;
        initial_Joints_accelerations_[i] = linear_accelaration;
        target_Joints_positions_[i] = target_pose_joints(i);
    }
}

void TASK2::init_reflexxes(){
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    RML_ = new ReflexxesAPI(DOF, CYCLE_TIME_IN_SECONDS);
    IP_  = new RMLPositionInputParameters(DOF);
    OP_  = new RMLPositionOutputParameters(DOF);
    update_reflexxes_parameters();

}
void TASK2::init_trajectory(){
    // Initializing the trajectory
    traj_.header.stamp = ros::Time::now();
    traj_.joint_names.resize(6);
    traj_.joint_names[0] = "shoulder_pan_joint";
    traj_.joint_names[1] = "shoulder_lift_joint";
    traj_.joint_names[2] = "elbow_joint";
    traj_.joint_names[3] = "wrist_1_joint";
    traj_.joint_names[4] = "wrist_2_joint";
    traj_.joint_names[5] = "wrist_3_joint";
    traj_.points.resize(1);
    traj_.points[0].positions.resize(6);
    for (int i = 0; i < DOF; i++)
    {
        traj_.points[0].positions[i] = initial_Joints_positions_[i];
    }

    traj_.points[0].time_from_start = ros::Duration(1.0);
}

bool TASK2::go_to_target(){
    // Calling the Reflexxes OTG algorithm
    RMLPositionFlags Flags;
    int ResultValue = RML_->RMLPosition(*IP_, OP_, Flags);

    if (ResultValue < 0)
    {
        printf("An error occurred (%d).\n", ResultValue );
        return false;
    }
    // Updating the initial parameters of the Reflexxes algorithm with the new values
    *IP_->CurrentPositionVector  =   *OP_->NewPositionVector  ;
    *IP_->CurrentVelocityVector  =   *OP_->NewVelocityVector  ;
    *IP_->CurrentAccelerationVector  =   *OP_->NewAccelerationVector  ;

    // Publishing the trajectory
    traj_.points.resize(1);
    traj_.points[0].positions.resize(DOF);
    for (int i = 0; i < DOF; i++) {
        traj_.points[0].positions[i] = OP_->NewPositionVector->VecData[i];
    }

    traj_.points[0].time_from_start = ros::Duration(1.0);
    joint_trajectory_pub_.publish(traj_);

    // Checking if the Reflexxes algorithm has finished
    if (ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) return true;
    else return false;
}

void TASK2::update_reflexxes_parameters(){

    // Adding reflexxes parameters
    for (int i = 0; i < DOF; i++) {
        // Initial joints positions, velocities and accelerations
        IP_->CurrentPositionVector->VecData[i] = initial_Joints_positions_[i];
        IP_->CurrentVelocityVector->VecData[i] = initial_Joints_velocities_[i];
        IP_->CurrentAccelerationVector->VecData[i] = initial_Joints_accelerations_[i];

        // Target joints positions, and velocities
        IP_->TargetPositionVector->VecData[i] = target_Joints_positions_[i];
        IP_->TargetVelocityVector->VecData[i] = initial_Joints_velocities_[i];  // Target velocity is the same as the initial velocity (for simplicity)
        IP_->SelectionVector->VecData [i] =  true;
        // Define kinematic limits
        IP_->MaxVelocityVector->VecData[i] = 1.0;
        IP_->MaxAccelerationVector->VecData[i] = 1.0;
        IP_->MaxJerkVector->VecData[i] = 1.0;
    }
}
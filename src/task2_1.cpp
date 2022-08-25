// used example in: http://www.reflexxes.ws/software/typeiirml/v1.2.6/docs/page__code_01__r_m_l_position_sample_application.html
#include "ur5_tasks/task2.h"

#define CYCLE_TIME_IN_SECONDS 0.1
#define DOF 6


TASK2::TASK2( ros::NodeHandle& nh) : nh(nh) {
    ros::Rate loop_rate(10);
    joint_trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);

    load_parameters();

    init_reflexxes_and_traj();
    update_reflexxes_parameters();

    // Starting the control loop
    int k = 1;
    bool goal_reached = false;
    auto start_time = std::chrono::system_clock::now();
    while (ros::ok()){
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start_time;
        if (elapsed_seconds.count() > 3.0 and !goal_reached)    // wait for 3 seconds until the robot reaches the initial pose
        {
            goal_reached = go_to_pose(k);
            k++;
        }
        else if (goal_reached) std::cout << "Goal reached" << std::endl;
        joint_trajectory_pub_.publish(traj_);
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

void TASK2::load_parameters(){


    int motion_number;
    nh.getParam("motion_type", motion_number);
    initial_Joints_positions_.resize(DOF);
    initial_Joints_velocities_.resize(DOF);
    initial_Joints_accelerations_.resize(DOF);
    target_Joints_positions_.resize(DOF);

    switch (motion_number) {
        case 1:
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
        case 2:
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
            inverse_kinematics(robot_description, pose1, pose2, linear_velocity, linear_accelaration);
            break;
        }
    }
    

    // std::cout << "robot_description_file: " << robot_description << std::endl;

    // double pos1;
    // nh.getParam("second_motion/pos1/x", pos1);
}
void TASK2::inverse_kinematics(std::string robot_description, std::vector<double> pose1, std::vector<double> pose2, double linear_velocity, double linear_accelaration){

    KDL::Tree my_tree;
    KDL::JntArray   initial_pose_cartesian(DOF),initial_pose_joints(DOF);
    KDL::JntArray   goal_pose_cartesian(DOF),goal_pose_joints(DOF);
    KDL::Frame desired_frame;

    if (!kdl_parser::treeFromString(robot_description, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    KDL::Chain chain;
    my_tree.getChain("world","tool0",chain);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    KDL::ChainIkSolverPos_NR ik_solver(chain, fk_solver, ik_solver_vel, 1000, 100);


    for (int i = 0; i < DOF; i++) {
        initial_pose_cartesian(i) = pose1[i];
        goal_pose_cartesian(i) = pose2[i];
    }
    // set the desired goal position in cartesian space
    // desired_frame.p.x(pose1[0]);
    // desired_frame.p.y(pose1[1]);
    // desired_frame.p.z(pose1[2]);
    ik_solver.CartToJnt(initial_pose_cartesian,desired_frame,initial_pose_joints);
    ik_solver.CartToJnt(goal_pose_cartesian,desired_frame,goal_pose_joints);


    for (int i = 0; i < DOF; i++){
        initial_Joints_positions_[i] = initial_pose_joints(i);
        initial_Joints_velocities_[i] = linear_velocity;
        initial_Joints_accelerations_[i] = linear_accelaration;
        target_Joints_positions_[i] = goal_pose_joints(i);
    }

}

void TASK2::init_reflexxes_and_traj(){
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    RML_ = new ReflexxesAPI(DOF, CYCLE_TIME_IN_SECONDS);
    IP_  = new RMLPositionInputParameters(DOF);
    OP_  = new RMLPositionOutputParameters(DOF);
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
// void TASK2::get_ur5_states(const sensor_msgs::JointState& states_msg){
//     for(int i=0;i<6;i++){
//         ur5_states_positions_.data[i]=states_msg.position[i];
//         ur5_states_velocities_.data[i]=states_msg.velocity[i];
//     }
// }

bool TASK2::go_to_pose(int k){
    // Calling the Reflexxes OTG algorithm
    ResultValue_ = RML_->RMLPosition(*IP_, OP_, Flags_);

    if (ResultValue_ < 0)
    {
        printf("An error occurred (%d).\n", ResultValue_ );
        return false;
    }

    *IP_->CurrentPositionVector  =   *OP_->NewPositionVector  ;
    *IP_->CurrentVelocityVector  =   *OP_->NewVelocityVector  ;
    *IP_->CurrentAccelerationVector  =   *OP_->NewAccelerationVector  ;

    std::cout << "new Position: " << OP_->NewPositionVector->VecData[0] << " " << OP_->NewPositionVector->VecData[1] << " " << OP_->NewPositionVector->VecData[2] << " " << OP_->NewPositionVector->VecData[3] << " " << OP_->NewPositionVector->VecData[4] << " " << OP_->NewPositionVector->VecData[5] << std::endl;
    // traj_.header.stamp = ros::Time::now();
    traj_.points.resize(0+1);
    traj_.points[0].positions.resize(6);
    traj_.points[0].positions[0]=(OP_->NewPositionVector->VecData[0]);
    traj_.points[0].positions[1]=(OP_->NewPositionVector->VecData[1]);
    traj_.points[0].positions[2]= (OP_->NewPositionVector->VecData[2]);
    traj_.points[0].positions[3]= (OP_->NewPositionVector->VecData[3]);
    traj_.points[0].positions[4]= (OP_->NewPositionVector->VecData[4]);
    traj_.points[0].positions[5]= (OP_->NewPositionVector->VecData[5]);

    traj_.points[0].time_from_start = ros::Duration((0+1)*1.0);
    joint_trajectory_pub_.publish(traj_);
    if (ResultValue_ == ReflexxesAPI::RML_FINAL_STATE_REACHED) return true;
    else return false;
}
void TASK2::update_reflexxes_parameters(){

    // Initialize trajectory with initial positions, velocities and accelerations
    for (int i = 0; i < DOF; i++) {
        IP_->CurrentPositionVector->VecData[i] = initial_Joints_positions_[i];
        IP_->CurrentVelocityVector->VecData[i] = initial_Joints_velocities_[i];
        IP_->CurrentAccelerationVector->VecData[i] = initial_Joints_accelerations_[i];
    }

    // Initialize trajectory with target positions, and velocities
    for ( int i = 0; i < DOF; i++ )
    {
        IP_->TargetPositionVector->VecData[i] = target_Joints_positions_[i];
        IP_->TargetVelocityVector->VecData[i] = 1.0;
        IP_->SelectionVector->VecData [i] =  true;
        // Define kinematic limits
        IP_->MaxVelocityVector->VecData[i] = 1.0;
        IP_->MaxAccelerationVector->VecData[i] = 1.0;
        IP_->MaxJerkVector->VecData[i] = 1.0;
    }
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "task2_planner");
    ros::NodeHandle nh;
    TASK2 task2(nh);
    return 0;
}

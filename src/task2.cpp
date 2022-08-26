// used example in: http://www.reflexxes.ws/software/typeiirml/v1.2.6/docs/page__code_01__r_m_l_position_sample_application.html
#include "ur5_tasks/task2.h"

#define CYCLE_TIME_IN_SECONDS 0.1
#define DOF 6


TASK2::TASK2( ros::NodeHandle& nh) : nh(nh) {
    q_init_ = KDL::JntArray(6);
    q_init_vel_ = KDL::JntArray(6);
    ros::Rate loop_rate(10);
    joint_trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);

    load_parameters();
    init_trajectory();

    // if (motion_number_==1) init_reflexxes();
    init_reflexxes();
    // Starting the control loop
    int k = 1;
    bool goal_reached = false;
    auto start_time = std::chrono::system_clock::now();
    while (ros::ok()){

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start_time;
        if (elapsed_seconds.count() > 3.0 and !goal_reached)    // wait for 3 seconds until the robot reaches the initial pose
        {
            // goal_reached = motion_number_== 1 ? go_to_joint_pose(k) : got_to_cartesian_pose(k);
            goal_reached = go_to_joint_pose(k);
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

    nh.getParam("motion_type", motion_number_);
    initial_Joints_positions_.resize(DOF);
    initial_Joints_velocities_.resize(DOF);
    initial_Joints_accelerations_.resize(DOF);
    target_Joints_positions_.resize(DOF);

    switch (motion_number_) {
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
            std::cout << "pose1: " << pose1[0] << " " << pose1[1] << " " << pose1[2] << std::endl;
            std::cout << "pose2: " << pose2[0] << " " << pose2[1] << " " << pose2[2] << std::endl;
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
    KDL::JntArray initial_pose_cartesian(DOF),initial_pose_joints(DOF),target_pose_joints(DOF);
    KDL::JntArray goal_pose_cartesian(DOF);
    target_pose_joints_ = KDL::JntArray(DOF);
    if (!kdl_parser::treeFromString(robot_description, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    KDL::Chain chain;
    my_tree.getChain("world","tool0",chain);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    ik_solver_ = new KDL::ChainIkSolverPos_NR(chain, fk_solver, ik_solver_vel, 1000, 100);


    // for (int i = 0; i < 3; i++) {
    //     initial_pose_cartesian(i) = pose1[i];
    //     goal_pose_cartesian(i) = pose2[i];
    // }
    // set the desired goal position in cartesian space
    desired_frame1_.p.x(pose1[0]);
    desired_frame1_.p.y(pose1[1]);
    desired_frame1_.p.z(pose1[2]);

    desired_frame2_.p.x(pose2[0]);
    desired_frame2_.p.y(pose2[1]);
    desired_frame2_.p.z(pose2[2]);
    std::cout << "desired_frame1_: " << desired_frame1_.p.x() << " " << desired_frame1_.p.y() << " " << desired_frame1_.p.z() << std::endl;
    std::cout << "desired_frame2_: " << desired_frame2_.p.x() << " " << desired_frame2_.p.y() << " " << desired_frame2_.p.z() << std::endl;
    ik_solver_->CartToJnt(q_init_,desired_frame1_,initial_pose_joints);
    // ik_solver_->CartToJnt(q_init_,desired_frame2_,target_pose_joints_);
    for (int i = 0; i < DOF; i++) {
    std::cout << "initial_pose_joints[" << i << "]: " << initial_pose_joints(i) << std::endl;
    }

    for (int i = 0; i < DOF; i++){
        initial_Joints_positions_[i] = initial_pose_joints(i);
        initial_Joints_velocities_[i] = linear_velocity;
        initial_Joints_accelerations_[i] = linear_accelaration;
        // target_Joints_positions_[i] = target_pose_joints_(i);
    }
    ik_solver_->CartToJnt(q_init_,desired_frame2_,target_pose_joints);

    for (int i = 0; i < DOF; i++){
    std::cout << "goal_pose_joints[" << i << "]: " << target_pose_joints(i) << std::endl;
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

bool TASK2::go_to_joint_pose(int k){
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
    traj_.points[0].positions.resize(DOF);
    for (int i = 0; i < DOF; i++)
    {
        traj_.points[0].positions[i] = OP_->NewPositionVector->VecData[i];
    }

    traj_.points[0].time_from_start = ros::Duration(1.0);
    joint_trajectory_pub_.publish(traj_);
    if (ResultValue_ == ReflexxesAPI::RML_FINAL_STATE_REACHED) return true;
    else return false;
}
bool TASK2::got_to_cartesian_pose(int k){
    // Calling the Reflexxes OTG algorithm
    ik_solver_->CartToJnt(q_init_,desired_frame2_,target_pose_joints_);
    // traj_.header.stamp = ros::Time::now();
    traj_.points.resize(0+1);
    traj_.points[0].positions.resize(6);
    for (int i = 0; i < DOF; i++)
    {
        traj_.points[0].positions[i] = target_pose_joints_.data(i);
        std::cout << "target_pose_joints_[" << i << "]: " << target_pose_joints_.data(i) << std::endl;
    }
    // traj_.points[0].positions[0]=(target_pose_joints_.data(0));
    // traj_.points[0].positions[1]=(target_pose_joints_.data(1));
    // traj_.points[0].positions[2]=(target_pose_joints_.data(2));
    // traj_.points[0].positions[3]=(target_pose_joints_.data(3));
    // traj_.points[0].positions[4]=(target_pose_joints_.data(4));
    // traj_.points[0].positions[5]=(target_pose_joints_.data(5));

    traj_.points[0].time_from_start = ros::Duration((0+1)*1.0);
    joint_trajectory_pub_.publish(traj_);
    if (q_init_.data[0]==target_Joints_positions_[0]) return true;
    else return false;
}
void TASK2::update_reflexxes_parameters(){

    // Initialize trajectory with from cuurent joint positions, velocities and accelerations
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

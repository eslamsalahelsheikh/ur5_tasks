// used example in: http://www.reflexxes.ws/software/typeiirml/v1.2.6/docs/page__code_01__r_m_l_position_sample_application.html
#include "ur5_tasks/task2.h"

#define CYCLE_TIME_IN_SECONDS 0.1
#define DOF 6


TASK2::TASK2( ros::NodeHandle& nh) : nh(nh) {
    ros::Rate loop_rate(10);
    joint_trajectory_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);

    std::vector<double> initial_Joints_positions = {-3.3, -1.5, 0.0, 1.5, 0.0, 0.0};
    std::vector<double> initial_Joints_velocities = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> initial_Joints_accelerations = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::vector<double> target_Joints_positions = std::vector<double>(DOF, 0.0);;


    init_reflexxes_and_traj(initial_Joints_positions);
    update_reflexxes_parameters(initial_Joints_positions, initial_Joints_velocities, initial_Joints_accelerations,target_Joints_positions);

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

void TASK2::init_reflexxes_and_traj(std::vector<double> initial_Joints_positions){
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
        traj_.points[0].positions[i] = initial_Joints_positions[i];
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
void TASK2::update_reflexxes_parameters(std::vector<double> initial_Joints_positions,std::vector<double>  initial_Joints_velocities,std::vector<double>  initial_Joints_accelerations, std::vector<double> target_Joints_positions){

    // Initialize trajectory with initial positions, velocities and accelerations
    for (int i = 0; i < DOF; i++) {
        IP_->CurrentPositionVector->VecData[i] = initial_Joints_positions[i];
        IP_->CurrentVelocityVector->VecData[i] = initial_Joints_velocities[i];
        IP_->CurrentAccelerationVector->VecData[i] = initial_Joints_accelerations[i];
    }

    // Initialize trajectory with target positions, and velocities
    for ( int i = 0; i < DOF; i++ )
    {
        IP_->TargetPositionVector->VecData[i] = target_Joints_positions[i];
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

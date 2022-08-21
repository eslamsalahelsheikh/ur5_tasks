// used example in: http://www.reflexxes.ws/software/typeiirml/v1.2.6/docs/page__code_01__r_m_l_position_sample_application.html
#include "ur5_tasks/task2.h"

#define CYCLE_TIME_IN_SECONDS 0.001
#define NUMBER_OF_DOFS 6


TASK2::TASK2( ros::NodeHandle& nh) : nh(nh) {
    ros::Rate loop_rate(10);
    joint_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);
    ur5_states_positions_= KDL::JntArray(NUMBER_OF_DOFS);
    ur5_states_velocities_ = KDL::JntArray(NUMBER_OF_DOFS);
    sub_js = nh.subscribe("/joint_states", 10, &TASK2::get_ur5_states, this);
    // pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1);




    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP  = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP  = new RMLPositionOutputParameters(NUMBER_OF_DOFS);


    // Initialize trajectory with initial positions, velocities and accelerations

    IP->CurrentPositionVector->VecData[0] =  -3.3;
    IP->CurrentPositionVector->VecData[1] =  -1.5;
    IP->CurrentPositionVector->VecData[2] =  0.0;
    IP->CurrentPositionVector->VecData[3] =  1.5;
    IP->CurrentPositionVector->VecData[4] =  0.0;
    IP->CurrentPositionVector->VecData[5] =  0.0;

    IP->CurrentVelocityVector->VecData[0] =  5.0;
    IP->CurrentVelocityVector->VecData[1] =  5.0;
    IP->CurrentVelocityVector->VecData[2] =  5.0;
    IP->CurrentVelocityVector->VecData[3] =  5.0;
    IP->CurrentVelocityVector->VecData[4] =  5.0;
    IP->CurrentVelocityVector->VecData[5] =  5.0;

    IP->CurrentAccelerationVector->VecData[0]= 1.0;
    IP->CurrentAccelerationVector->VecData[1]= 1.0;
    IP->CurrentAccelerationVector->VecData[2]= 1.0;
    IP->CurrentAccelerationVector->VecData[3]= 1.0;
    IP->CurrentAccelerationVector->VecData[4]= 1.0;
    IP->CurrentAccelerationVector->VecData[5]= 1.0;

    // Initialize trajectory with target positions, velocities and accelerations
    for ( int i = 0; i < 6; i++ )
    {
        IP->TargetVelocityVector->VecData[i] = 1.0;
    }
    // Define kinematic limits (from URDF file)
    for ( int i = 0; i < 6; i++ )
    {
        IP->MaxVelocityVector->VecData[i] = 10.0;
        IP->MaxAccelerationVector->VecData[i] = 1.0;
        IP->MaxJerkVector->VecData[i] = 1.0;
        IP->SelectionVector->VecData [i] =  true;

    }

    IP->TargetPositionVector->VecData[0] = 0.0;
    IP->TargetPositionVector->VecData[1] = -1.5;
    IP->TargetPositionVector->VecData[2] = 0.0;
    IP->TargetPositionVector->VecData[3] = -1.5;
    IP->TargetPositionVector->VecData[4] = 0.0;
    IP->TargetPositionVector->VecData[5] = 0.0;

    // ********************************************************************
    // Starting the control loop
    trajectory_msgs::JointTrajectory traj;

    traj.header.stamp = ros::Time::now();
    traj.joint_names.resize(6);
    traj.joint_names[0] = "shoulder_pan_joint";
    traj.joint_names[1] = "shoulder_lift_joint";
    traj.joint_names[2] = "elbow_joint";
    traj.joint_names[3] = "wrist_1_joint";
    traj.joint_names[4] = "wrist_2_joint";
    traj.joint_names[5] = "wrist_3_joint";
    // traj.points.resize(1);
    // traj.points[0].positions.resize(6);
    // traj.points[0].positions[0]=  0;
    // traj.points[0].positions[1]=  0;
    // traj.points[0].positions[2]=  0.0;
    // traj.points[0].positions[3]=  0;
    // traj.points[0].positions[4]=  0.0;
    // traj.points[0].positions[5]=  0.0;


    // traj.points[0].time_from_start = ros::Duration(1.0);

    int k = 1;
    while (ros::ok()){

        // Calling the Reflexxes OTG algorithm
    if (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {

        ResultValue = RML->RMLPosition(*IP, OP, Flags);

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }
        // update the current positions, velocities (from actutal joint states) and accelerations (from model)
        for ( int i = 0; i < 6; i++ )
        {
            IP->CurrentPositionVector->VecData[i] = ur5_states_positions_.data[i];
            IP->CurrentVelocityVector->VecData[i] = ur5_states_velocities_.data[i];
        }
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

        std::cout << "new Position: " << OP->NewPositionVector->VecData[0] << " " << OP->NewPositionVector->VecData[1] << " " << OP->NewPositionVector->VecData[2] << " " << OP->NewPositionVector->VecData[3] << " " << OP->NewPositionVector->VecData[4] << " " << OP->NewPositionVector->VecData[5] << std::endl;
        traj.header.stamp = ros::Time::now();
        traj.points.resize(k+1);
        traj.points[k].positions.resize(6);
        traj.points[k].positions[0]=(OP->NewPositionVector->VecData[0]);
        traj.points[k].positions[1]=(OP->NewPositionVector->VecData[1]);
        traj.points[k].positions[2]= (OP->NewPositionVector->VecData[2]);
        traj.points[k].positions[3]= (OP->NewPositionVector->VecData[3]);
        traj.points[k].positions[4]= (OP->NewPositionVector->VecData[4]);
        traj.points[k].positions[5]= (OP->NewPositionVector->VecData[5]);

        traj.points[k].time_from_start = ros::Duration((k+1)*1.0);
        k++;
        joint_trajectory_pub.publish(traj);
    }
    else {
            printf("The final state of the motion was reached.\n");
        }
    ros::spinOnce();
    loop_rate.sleep();
    }

};


TASK2::~TASK2()
{
    delete RML;
    delete IP;
    delete OP;
}

void TASK2::get_ur5_states(const sensor_msgs::JointState& states_msg){
    for(int i=0;i<6;i++){
        ur5_states_positions_.data[i]=states_msg.position[i];
        ur5_states_velocities_.data[i]=states_msg.velocity[i];
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "task2_planner");
    ros::NodeHandle nh;
    TASK2 task2(nh);
    return 0;
}

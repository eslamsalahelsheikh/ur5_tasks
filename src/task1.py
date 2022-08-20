import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import *
import math

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def task1_sinwave():
    rospy.init_node("task1_sinwave", anonymous=True)
    rate = rospy.Rate(1) # 1hz (a little slow but to just to see the sine wave)
    pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

    # initialize sine array
    sine_angles = []
    angle = 0.0
    while angle <=180:
        sine_angles.append(math.sin(angle))
        angle += 20


    # put the robot in a safe position first
    StartingAngles =[-3.3463198132812973e-05, -1.570651088413829, -2.524827304296195e-05, -1.5706490570078604, 8.177598407492041e-05, 4.551892913877959e-06]
    starting_trajectory = JointTrajectory()
    starting_trajectory.joint_names = JOINT_NAMES

    starting_trajectory.points = []
    point = JointTrajectoryPoint(positions=StartingAngles, velocities=[0]*6, time_from_start=rospy.Duration(2))
    starting_trajectory.points.append(point)
    pub.publish(starting_trajectory)


    iteration = 0
    joint_num = 0
    trajectory = starting_trajectory
    AxisAngles = StartingAngles.copy()
    while not rospy.is_shutdown():
        if iteration == len (sine_angles):
            iteration = 0
            joint_num += 1
            if joint_num > 5: joint_num = 0 # restart loop
            AxisAngles = StartingAngles.copy()

        else:
            AxisAngles[joint_num] -= sine_angles[iteration]

        point = JointTrajectoryPoint(positions=AxisAngles, velocities=[0]*6, time_from_start=rospy.Duration(2))
        trajectory.points = []
        trajectory.points.append(point)

        pub.publish(trajectory)
        iteration += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        task1_sinwave()
    except rospy.ROSInterruptException:
        pass
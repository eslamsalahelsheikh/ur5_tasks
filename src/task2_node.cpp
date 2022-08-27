#include "ur5_tasks/task2.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "task2_planner");
    ros::NodeHandle nh;
    TASK2 task2(nh);
    return 0;
}

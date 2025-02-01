#include "planning.h"
#include <ros/ros.h>
#include <vector>
#include <unistd.h> // Para sleep()

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_execution");
    ros::NodeHandle nh;
    my_planning::MyPlanningClass planner(nh);

    ROS_INFO("Starting pick sequence");
    //planner.goToPick();  
    system("rosrun planning run 4");

    ros::Duration(100).sleep(); // Espera hasta el minuto 2:10
    ROS_INFO("Executing Caja 1");
    //planner.goToCaja1();  
    system("rosrun planning run 1");

    ros::Duration(200).sleep(); // Espera hasta el minuto 5:30
    ROS_INFO("Executing Caja 2");
    //planner.goToCaja2();  
    system("rosrun planning run 2");

    ros::Duration(194).sleep(); // Espera hasta el minuto 8:44
    ROS_INFO("Executing Caja 3");
    //planner.goToCaja3();  
    system("rosrun planning run 3");

    ROS_INFO("Pick sequence completed");
    return 0;
}

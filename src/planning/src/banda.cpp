#include <ros/ros.h>
#include <unistd.h> // Para sleep()

class MyPlanningClass {
public:
    void setConveyorPower(int power) {
        std::string command = "rosservice call /conveyor/control \"{power: " + std::to_string(power) + "}\"";
        system(command.c_str());
        ROS_INFO("Conveyor power set to: %d", power);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "banda");
    ros::NodeHandle nh;

    MyPlanningClass conveyor;
    
    while (ros::ok()) {
        conveyor.setConveyorPower(5);
        ros::Duration(96).sleep(); // 1 min 36 seg

        conveyor.setConveyorPower(1);
        ros::Duration(8).sleep(); // 8 seg

        conveyor.setConveyorPower(0);
        ros::Duration(60).sleep(); // 1 min
    }

    return 0;
}

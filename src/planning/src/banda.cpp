#include <ros/ros.h>
#include <unistd.h> // Para sleep()

class MyPlanningClass {
public:
    void startConveyor() {
        system("rosservice call /conveyor/control \"{power: 10}\"");
    }

    void stopConveyor() {
        system("rosservice call /conveyor/control \"{power: 0}\"");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "banda");
    ros::NodeHandle nh;

    MyPlanningClass conveyor;
    
    while (ros::ok()) {
        conveyor.startConveyor();
        ros::Duration(79).sleep(); // Espera 1 min 19 seg
        conveyor.stopConveyor();
        ros::Duration(3).sleep(); // Detiene por 3 segundos
    }

    return 0;
}
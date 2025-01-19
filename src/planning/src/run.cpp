#include <planning.h>
#include <cstdlib> // Para atof y atoi

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_interfacing");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    if (argc < 2) // Asegúrate de que haya al menos un argumento
    {
        ROS_INFO(" ");
        ROS_INFO("\tUsage:");
        ROS_INFO(" ");
        ROS_INFO("\trosrun planning run <n> [x y z]");
        return 1;
    }

    my_planning::MyPlanningClass my_planning_;
    my_planning_.resetValues(); // Llamada fuera del switch para inicializar siempre

    int selection = atoi(argv[1]); // Primer argumento como número del caso

    switch (selection)
    {
        case 1:
            my_planning_.OpenGripper();
            break;
        case 2:
            my_planning_.CloseGripper();
            break;
        case 3:
            my_planning_.cartesianPath2();
            break;
        case 4:
            my_planning_.goToJointArticulateState();
            break;
        case 5:
            my_planning_.goToInitialState();
            break;
        default:
            ROS_WARN("Invalid option. Please select a valid number between 1 and 6.");
            break;
    }

    spinner.stop();
    return 0;
}

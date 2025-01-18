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
        ROS_INFO("\trosrun planning run <n> [x y z orientation]");
        return 1;
    }

    my_planning::MyPlanningClass my_planning_;
    my_planning_.resetValues(); // Llamada fuera del switch para inicializar siempre

    int selection = atoi(argv[1]); // Primer argumento como número del caso

    switch (selection)
    {
        case 1:
            my_planning_.cartesianPath();
            break;
        case 2:
            my_planning_.goToInitialState();
            break;
        case 3:
            if (argc == 6) // Asegúrate de que haya 5 argumentos para x, y, z y orientación
            {
                double x = atof(argv[2]);
                double y = atof(argv[3]);
                double z = atof(argv[4]);
                double orientation = atof(argv[5]);

                ROS_INFO("Running cartesianPath with x=%.2f, y=%.2f, z=%.2f, orientation=%.2f", x, y, z, orientation);
                my_planning_.goToPosition(x, y, z, orientation);
            }
            else
            {
                ROS_WARN("Invalid arguments. Usage for case 3: rosrun planning run 3 <x> <y> <z> <orientation>");
            }
            break;
        case 4:
            my_planning_.addObjects();
            break;
        case 5:
            my_planning_.cartesianPath2();
            break;
        default:
            ROS_WARN("Invalid option. Please select a valid number between 1 and 6.");
            break;
    }

    spinner.stop();
    return 0;
}

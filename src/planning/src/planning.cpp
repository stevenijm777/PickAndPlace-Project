#include <planning.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace my_planning
{
void MyPlanningClass::goToPoseGoal()
    {
        move_group.setPoseTarget(target_pose1);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) // execute
            throw std::runtime_error("No plan found");

        move_group.move(); // blocking
    }

    void MyPlanningClass::goToPoseGoal(geometry_msgs::Pose &pose)
    {
        move_group.setPoseTarget(pose);
        ros::Duration(0.5).sleep();
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        /*while (!success) //keep trying until a plan is found
        {

            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }*/

        if (!success) // execute
            throw std::runtime_error("No plan found");

        move_group.move(); // blocking
    }

    void MyPlanningClass::goToJointState(double q1, double q2, double q3, double q4, double q5, double q6)
    {
        robot_state::RobotState current_state = *move_group.getCurrentState();
        std::vector<double> joint_positions;
        joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
        current_state.copyJointGroupPositions(joint_model_group, joint_positions);

        joint_positions[0] = q1;
        joint_positions[1] = q2;
        joint_positions[2] = q3;
        joint_positions[3] = q4;
        joint_positions[4] = q5;
        joint_positions[5] = q6;

        move_group.setJointValueTarget(joint_positions);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
            throw std::runtime_error("No plan found");

        move_group.move(); // blocking
    }

    void MyPlanningClass::goToInitialState()
    {
        robot_state::RobotState current_state = *move_group.getCurrentState();
        std::vector<double> joint_positions;
        joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
        current_state.copyJointGroupPositions(joint_model_group, joint_positions);
// initial 
//        joint_positions[0] = -0.243;
//        joint_positions[1] = 1.266;
//        joint_positions[2] = -0.103;
//        joint_positions[3] = 0.198;
//        joint_positions[4] = -0.035;
//        joint_positions[5] = -0.175;
//        joint_positions[6] = 0.063;
        joint_positions[0] = 0;
        joint_positions[1] = 0;
        joint_positions[2] = 0;
        joint_positions[3] = 0;
        joint_positions[4] = 0;
        joint_positions[5] = 0;

        move_group.setJointValueTarget(joint_positions);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
            throw std::runtime_error("No plan found");
        move_group.move(); // blocking
    }

    void MyPlanningClass::cartesianPath()
    {
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose1);

        geometry_msgs::Pose target_pose2 = target_pose1;
        target_pose2.position.x = 0.5;
        target_pose2.position.y = -0.5;
        target_pose2.position.z = 0.6;
        waypoints.push_back(target_pose2);

        target_pose2.position.z += 0.5;
        waypoints.push_back(target_pose2);

        target_pose2.position.y += 1.0;
        waypoints.push_back(target_pose2);

        target_pose2.position.z -=0.5;

        move_group.setMaxVelocityScalingFactor(0.1);

        // We want the Cartesian path to be interpolated at a resolution of 1 cm
        // which is why we will specify 0.01 as the max step in Cartesian
        // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
        // Warning - disabling the jump threshold while operating real hardware can cause
        // large unpredictable motions of redundant joints and could be a safety issue
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        move_group.move();
        ROS_INFO_STREAM("Percentage of path followed: " << fraction);
    }

    void MyPlanningClass::resetValues()
    {
        // set the start state and operational speed
        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(1.0);
    }

    void MyPlanningClass::makeTable()
    {
        moveit_msgs::CollisionObject table;

        // Establecer el marco de referencia relativo
        table.header.frame_id = move_group.getPlanningFrame();
        table.id = "cafe_table";

        // Definir la geometría de la mesa
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.913; // Largo de la mesa
        primitive.dimensions[1] = 0.913; // Ancho de la mesa
        primitive.dimensions[2] = 0.720; // Altura de la mesa

        // Definir la pose de la mesa
        geometry_msgs::Pose table_pose;
        table_pose.position.x = 1.0;  // Coordenada x
        table_pose.position.y = 0.0;  // Coordenada y
        table_pose.position.z = -0.6; // Coordenada z (mitad de la altura)

        // Agregar geometría y pose al objeto de colisión
        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;

        // Agregar el objeto de colisión al entorno de planificación
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(table);
        ros::Duration(2).sleep();
        virtual_world.addCollisionObjects(collision_objects);
        ROS_INFO_STREAM("Added: cafe_table");
    }

    void MyPlanningClass::makeBox(std::string blk_name, double *pose)
    {
        moveit_msgs::CollisionObject box;
        // set the relative frame
        box.header.frame_id = move_group.getPlanningFrame();
        box.id = blk_name;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.5;
        primitive.dimensions[1] = 0.3;
        primitive.dimensions[2] = 0.2;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = pose[0];
        box_pose.position.y = pose[1];
        box_pose.position.z = pose[2];

        box.primitives.push_back(primitive);
        box.primitive_poses.push_back(box_pose);
        box.operation = box.ADD;

        std::vector<moveit_msgs::CollisionObject> collisionObjects;
        collisionObjects.push_back(box);
        ros::Duration(2).sleep();
        virtual_world.addCollisionObjects(collisionObjects);
        ROS_INFO_STREAM("Added: " << blk_name);
    }

    void MyPlanningClass::addObjects()
    {
        double box_pose1[3] = {-0.80, -0.25, -0.8};
        makeBox("block_1", box_pose1);

        double box_pose2[3] = {-0.8, 0.1, -0.8};
        makeBox("block_2", box_pose2);

        double box_pose3[3] = {1.19, 0.17, -0.8};
        makeBox("block_3", box_pose2);

        // Add table
        //makeTable();
    }

    void MyPlanningClass::removeObjects()
    {
        std::vector<std::string> object_ids;
        // object_ids.push_back("block_1");
        // object_ids.push_back("block_2");
        object_ids.push_back("cafe_table");
        virtual_world.removeCollisionObjects(object_ids);
    }

    void MyPlanningClass::goToPosition(double x, double y, double z)
    {
        std::vector<geometry_msgs::Pose> waypoints;

        // Pose inicial
        geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose; // Obten la posicion actual
        waypoints.push_back(start_pose);

        // Pose objetivo
        geometry_msgs::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        // Manten la orientacion inicial para este ejemplo
        // target_pose.orientation = start_pose.orientation;

        waypoints.push_back(target_pose);

        // Configuracion de la interpolacion cartesiana
        move_group.setMaxVelocityScalingFactor(0.1); // Escalado de velocidad
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01; // Resolucion de interpolacion

        // Generar trayectoria cartesiana
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        if (fraction > 0.95)
        { // Si mas del 95% de la trayectoria es alcanzable
            ROS_INFO_STREAM("Trayectoria generada con exito. Fraccion alcanzada: " << fraction);
            move_group.execute(trajectory); // Ejecutar la trayectoria generada
        }
        else
        {
            ROS_WARN_STREAM("Trayectoria incompleta. Fraccion alcanzada:" << fraction);
        }
    }

        void MyPlanningClass::cartesianPath2()
    {
        std::vector<geometry_msgs::Pose> waypoints;

        // Pose inicial
        geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose; // Obten la posicion actual
        waypoints.push_back(start_pose);

        // Pose objetivo
        geometry_msgs::Pose target_pose;
        target_pose.position.x = 1.09;
        target_pose.position.y = 0.15;
        target_pose.position.z = 0.32;

        // Manten la orientacion inicial para este ejemplo
        // target_pose.orientation = start_pose.orientation;

        waypoints.push_back(target_pose);

        // Configuracion de la interpolacion cartesiana
        move_group.setMaxVelocityScalingFactor(0.1); // Escalado de velocidad
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01; // Resolucion de interpolacion

        // Generar trayectoria cartesiana
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        if (fraction > 0.8)
        { // Si mas del 95% de la trayectoria es alcanzable
            ROS_INFO_STREAM("Trayectoria generada con exito. Fraccion alcanzada: " << fraction);
            move_group.execute(trajectory); // Ejecutar la trayectoria generada
        }
        else
        {
            ROS_WARN_STREAM("Trayectoria incompleta. Fraccion alcanzada:" << fraction);
        }
}

void MyPlanningClass::goToJointArticulateState()
{
    // Configuraciones articulares: izquierda bajo -> izquierda arriba -> derecha arriba -> derecha abajo
    std::vector<std::vector<double>> joint_positions_list = {
        {0, 0, 0, 0, 0, 0, 0 },
        {-1.709, -0.333, 0.480, 0.744, -0.441, 0.843, 0.177}, // Izquierda arriba
    };

    for (const auto& target_joint_positions : joint_positions_list) {
        goToJointState(
            target_joint_positions[0],
            target_joint_positions[1],
            target_joint_positions[2],
            target_joint_positions[3],
            target_joint_positions[4],
            target_joint_positions[5]
        );
        ros::Duration(1.0).sleep();
    
    }
}

void MyPlanningClass::goToCaja1(){
    std::vector<std::vector<double>> joint_positions_list = {
        {-3.049, -0.255, 0.092, -0.167, -0.190, 0.275, -0.024}, // caja 1
        {-1.709, -0.333, 0.480, 0.744, -0.441, 0.843, 0.177}, // ready to pick
    };

    for (const auto& target_joint_positions : joint_positions_list) {
        goToJointState(
            target_joint_positions[0],
            target_joint_positions[1],
            target_joint_positions[2],
            target_joint_positions[3],
            target_joint_positions[4],
            target_joint_positions[5]
        );
        ros::Duration(1.0).sleep();
    
    }
}


void MyPlanningClass::goRightPosition(double x, double y, double z){
        std::vector<geometry_msgs::Pose> waypoints;

        // Pose inicial
        geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose; // Obten la posicion actual
        waypoints.push_back(start_pose);

        // Pose objetivo
        geometry_msgs::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        // Manten la orientacion inicial para este ejemplo
        target_pose.orientation = start_pose.orientation;

        waypoints.push_back(target_pose);

        // Configuracion de la interpolacion cartesiana
        move_group.setMaxVelocityScalingFactor(0.1); // Escalado de velocidad
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01; // Resolucion de interpolacion

        // Generar trayectoria cartesiana
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        if (fraction > 0.95)
        { // Si mas del 95% de la trayectoria es alcanzable
            ROS_INFO_STREAM("Trayectoria generada con exito. Fraccion alcanzada: " << fraction);
            move_group.execute(trajectory); // Ejecutar la trayectoria generada
        }
        else
        {
            ROS_WARN_STREAM("Trayectoria incompleta. Fraccion alcanzada:" << fraction);
        }
}

void MyPlanningClass::controlGripper()
{
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<std_msgs::Int32>("/gripper_control", 10);
    // Comando para abrir el gripper (1: abrir, 0: cerrar)
    std_msgs::Int32 msg;
    msg.data = 1; // Cambia a 0 para cerrar el gripper
    gripper_pub.publish(msg);
    ros::Duration(2.0).sleep(); // 
    msg.data = 0;
    gripper_pub.publish(msg);
    ros::Duration(2.0).sleep(); // 
}

void MyPlanningClass::OpenGripper()
{
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<std_msgs::Int32>("/gripper_control", 10);
    // Comando para abrir el gripper (1: abrir, 0: cerrar)
    std_msgs::Int32 msg;
    msg.data = 1;
    gripper_pub.publish(msg);
    ROS_INFO("Mensaje enviado: %d", msg.data);
    ros::Duration(1.0).sleep();
}

void MyPlanningClass::CloseGripper()
{
    ros::NodeHandle nh;
    ros::Publisher gripper_pub = nh.advertise<std_msgs::Int32>("/gripper_control", 10);
    // Comando para abrir el gripper (1: abrir, 0: cerrar)
    std_msgs::Int32 msg;
    msg.data = 0;
    gripper_pub.publish(msg);
    ROS_INFO("Mensaje enviado: %d", msg.data);
    ros::Duration(1.0).sleep();
}

void MyPlanningClass::PickAndPlace()
{
    goToPosition(1, 0.5, 0.5);
    goRightPosition(1, 0.5, 1.0);
    goRightPosition(1, -0.5, 1.0);
    goRightPosition(1, -0.5, 0.5);
}

}

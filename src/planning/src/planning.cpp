#include <planning.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

namespace my_planning
{
    MyPlanningClass::MyPlanningClass(ros::NodeHandle& nh) : move_group(PLANNING_GROUP), nh_(nh)
    {
        // Inicializa el publicador en el constructor
        gripper_pub = nh.advertise<std_msgs::Int32>("/gripper_control", 10);
    }

    void MyPlanningClass::OpenGripper()
    {
       // Comando para abrir el gripper (1: abrir, 0: cerrar)
        std_msgs::Int32 msg;
        msg.data = 1;
        // Publica el mensaje
        if (gripper_pub) // Asegurate de que el publicador esta inicializado
        {
            gripper_pub.publish(msg);
            ROS_INFO("Mensaje enviado: %d", msg.data);
        }
        else
        {
            ROS_ERROR("El publicador no esta inicializado correctamente");
        }
        ros::Duration(1.0).sleep();
    }

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
        joint_positions[0] = -0.243;
        joint_positions[1] = 1.266;
        joint_positions[2] = -0.103;
        joint_positions[3] = 0.198;
        joint_positions[4] = -0.035;
        joint_positions[5] = -0.175;
        joint_positions[6] = 0.063;


        move_group.setJointValueTarget(joint_positions);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success)
            throw std::runtime_error("No plan found");
        move_group.move(); // blocking
    }

    void MyPlanningClass::resetValues()
    {
        // set the start state and operational speed
        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(1.0);
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

        double box_pose3[3] = {0.8, 0.17, -0.8};
        makeBox("block_3", box_pose2);

        // Add table
        //makeTable();
    }

void MyPlanningClass::goToArticulateList(const std::vector<std::vector<double>>& joint_positions_list) {
    for (const auto& target_joint_positions : joint_positions_list) {
        goToJointState(
            target_joint_positions[0],
            target_joint_positions[1],
            target_joint_positions[2],
            target_joint_positions[3],
            target_joint_positions[4],
            target_joint_positions[5]
        );
        // Ajustar el tiempo de espera en función de la posición
        if (target_joint_positions[5] >= 0.5) {
            ros::Duration(2).sleep();
        } else {
            ros::Duration(0.5).sleep();
        }
    }
}

void MyPlanningClass::goToPick()
{
    // Configuraciones articulares: izquierda bajo -> izquierda arriba -> derecha arriba -> derecha abajo
    std::vector<std::vector<double>> joint_positions_list = {
        {0, 0, 0, 0, 0, 0, 0 },
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202} // Go to center
    };
    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
}

void MyPlanningClass::goToCaja1(){
    std::vector<std::vector<double>> joint_positions_list = {
        {-1.653, -0.754, 0.248, 0.893, -0.266, 0.836, 0,202}, //sube primero
        {-2.943, -0.493, 0.007, 0.800, -0.191, 1.439, 0.208},
        {-3.049, -0.255, 0.092, -0.167, -0.190, 0.275, 0.6}, // caja 1
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202}, // ready to pick
    };

    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
}

void MyPlanningClass::goToCaja2(){
    std::vector<std::vector<double>> joint_positions_list = {
        {-1.653, -0.754, 0.248, 0.893, -0.266, 0.836, 0,202}, //sube primero
        {-3.049, 0.382, 0.077, 0.259, 2.976, -1.059, 0.6}, // caja 2
        {-3.050, -0.242, 0.011, 0.046, 2.976, -1.522, 0.204}, //sube 
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202}, // ready to pick
    };

    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
}

void MyPlanningClass::goToCaja3(){
    std::vector<std::vector<double>> joint_positions_list = {
        {-1.653, -0.754, 0.248, 0.893, -0.266, 0.836, 0,202}, //sube primero
        {0.183, -0.057, 0.077, 0.259, 2.976, -1.544, 0.6}, // caja 3
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202}, // ready to pick
    };

    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
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

}

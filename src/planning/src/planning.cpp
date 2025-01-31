#include <planning.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <cstdlib>
namespace my_planning
{
    MyPlanningClass::MyPlanningClass(ros::NodeHandle& nh) : move_group(PLANNING_GROUP), nh_(nh)
    {
        // Inicializa el publicador en el constructor
        //gripper_pub = nh.advertise<std_msgs::Int32>("/gripper_control", 10);
        gripper_left_pub = nh_.advertise<std_msgs::Float64>("/robot/electric_gripper_controller/joints/right_gripper_l_finger_controller/command", 10);
        gripper_right_pub = nh_.advertise<std_msgs::Float64>("/robot/electric_gripper_controller/joints/right_gripper_r_finger_controller/command", 10);
    }

    void MyPlanningClass::OpenGripper()
    {
        //std_msgs::Float64 msg_left, msg_right;

        // Comando para abrir: Left -> 1.0, Right -> -1.0
        //msg_left.data = 1.0;
        //msg_right.data = -1.0;

        // Publicar los comandos
        //gripper_left_pub.publish(msg_left);
        //gripper_right_pub.publish(msg_right);   
        //ROS_INFO("Enviando comando para abrir gripper: Left = %f, Right = %f", msg_left.data, msg_right.data);     
        //ROS_INFO("Publicando en el tema izquierdo: %s", gripper_left_pub.getTopic().c_str());
        //ROS_INFO("Publicando en el tema derecho: %s", gripper_right_pub.getTopic().c_str());
        //ros::spinOnce();  // Permite procesar callbacks inmediatamente
        // permitir el movimiento del gripper usando controlGripper
        //system("rostopic pub -1 /gripper_control std_msgs/Int32 \"data: 1\"");
        // usando electric_gripper para mayor rango de apertura
        //pub para abrir el left 
        system("rostopic pub -1 /robot/electric_gripper_controller/joints/right_gripper_l_finger_controller/command std_msgs/Float64 \"data: 1.0\"");
        //pub para abrir el right
        system("rostopic pub -1 /robot/electric_gripper_controller/joints/right_gripper_r_finger_controller/command std_msgs/Float64 \"data: -1.0\"");

        ros::Duration(0.5).sleep();
    }

    void MyPlanningClass::CloseGripper()
    {
        //ros::NodeHandle nh;
        //ros::Publisher gripper_pub = nh.advertise<std_msgs::Int32>("/gripper_control", 10);
        //// Comando para abrir el gripper (1: abrir, 0: cerrar)
        //std_msgs::Int32 msg;
        //msg.data = 0;
        //gripper_pub.publish(msg);
        //ROS_INFO("Mensaje enviado: %d", msg.data);
        //pub para abrir el left 
        system("rostopic pub -1 /robot/electric_gripper_controller/joints/right_gripper_l_finger_controller/command std_msgs/Float64 \"data: 0.0\"");
        //pub para abrir el right
        system("rostopic pub -1 /robot/electric_gripper_controller/joints/right_gripper_r_finger_controller/command std_msgs/Float64 \"data: 0.0\"");
        ros::Duration(0.5).sleep();
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

    void MyPlanningClass::goToJointState(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
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
        joint_positions[6] = q7;

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
        // Ajustar el tiempo de espera en función de la posición
        if (target_joint_positions[6] >= 0.5 or target_joint_positions[6] <=0.230) {            
            ros::Duration(0.5).sleep();
            OpenGripper();
            ros::Duration(0.5).sleep();
        } else {
            CloseGripper();
            ros::Duration(0.5).sleep();
        }
        goToJointState(
            target_joint_positions[0],
            target_joint_positions[1],
            target_joint_positions[2],
            target_joint_positions[3],
            target_joint_positions[4],
            target_joint_positions[5],
            target_joint_positions[6]
        );
    }
}

void MyPlanningClass::goToPick()
{
    // Configuraciones articulares: izquierda bajo -> izquierda arriba -> derecha arriba -> derecha abajo
    std::vector<std::vector<double>> joint_positions_list = {
        {0, 0, 0, 0, 0, 0, 0.202 },
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202} // Go to center
    };
    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
}

void MyPlanningClass::goToCaja1(){
    std::vector<std::vector<double>> joint_positions_list = {
        {-1.662, -0.430, 0.190, 0.800, -0.266, 1.137, 0.402}, //sube primero
        {-2.8, -0.430, 0.190, 0.800, -0.266, 1.137, 0.402}, // a un lado 
        {-3.049, -0.231, 0.145, 1.049, -0.008, 0.806, 0.6}, // caja 1
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202}, // ready to pick
    };
    

    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
}

void MyPlanningClass::goToCaja2(){
    std::vector<std::vector<double>> joint_positions_list = {
        {-1.662, -0.430, 0.190, 0.800, -0.266, 1.137, 0.402}, //sube primero
        {-3.049, 0.382, 0.077, 0.259, 2.976, -1.059, 0.6}, // caja 2
        {-3.050, -0.242, 0.011, 0.046, 2.976, -1.522, 0.402}, //sube 
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202}, // ready to pick
    };

    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
}

void MyPlanningClass::goToCaja3(){
    std::vector<std::vector<double>> joint_positions_list = {
        {-1.662, -0.430, 0.190, 0.800, -0.266, 1.137, 0.402}, //sube primero
        {0.183, -0.057, 0.077, 0.259, 2.976, -1.544, 0.6}, // caja 3
        {-1.644, -0.385, 0.247, 0.861, -0.260, 0.847, 0.202}, // ready to pick
    };

    // Llamada a la función genérica
    goToArticulateList(joint_positions_list);
}

void MyPlanningClass::stopConveyor(){
    system("rosservice call /conveyor/control \"{power: 0}\"");
}

}

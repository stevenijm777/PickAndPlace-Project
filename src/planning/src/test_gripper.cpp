#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_gripper");
    ros::NodeHandle nh;
    
    ros::Publisher left_pub = nh.advertise<std_msgs::Float64>(
        "/robot/electric_gripper_controller/joints/right_gripper_l_finger_controller/command", 
        10
    );
    
    ros::Publisher right_pub = nh.advertise<std_msgs::Float64>(
        "/robot/electric_gripper_controller/joints/right_gripper_r_finger_controller/command", 
        10
    );

    std_msgs::Float64 msg_left, msg_right;
    msg_left.data = 1.0;
    msg_right.data = -1.0;

    ROS_INFO("Publicando mensajes de prueba...");
    left_pub.publish(msg_left);
    right_pub.publish(msg_right);
    
    ros::Duration(1.0).sleep(); // Esperar para publicar
    ros::spinOnce();
    
    ROS_INFO("Mensajes enviados. Verifica con rostopic echo.");
    ros::Duration(2.0).sleep();
    return 0;
}
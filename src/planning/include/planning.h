#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

namespace my_planning
{
    class MyPlanningClass
    {
        public:
            MyPlanningClass(ros::NodeHandle& nh);
            MyPlanningClass(): move_group(PLANNING_GROUP)
            {
                target_pose1.orientation.w = 1.0;
                target_pose1.position.x = 0.38;
                target_pose1.position.y = -0.2;
                target_pose1.position.z = 0.65;

                move_group.allowReplanning(true);
                move_group.setNumPlanningAttempts(10);
            }
            void goToJointState(double q1, double q2, double q3, double q4, double q5, double q6);
            void goToPoseGoal(geometry_msgs::Pose &pose);
            void goToPoseGoal();
            void resetValues();
            void addObjects();
            void goToInitialState();
            void makeBox(std::string blk_name, double *pose);
            void goToPick();
            void OpenGripper();
            void CloseGripper();
            void goToCaja1();
            void goToCaja2();
            void goToCaja3();
            void goToArticulateList(const std::vector<std::vector<double>>& joint_positions_list);


        private:
            const std::string PLANNING_GROUP = "right_arm";
            ros::Publisher gripper_left_pub; // Publicador persistente
            ros::Publisher gripper_right_pub;
            moveit::planning_interface::MoveGroupInterface move_group;
            moveit::planning_interface::PlanningSceneInterface virtual_world;
            const robot_state::JointModelGroup* joint_model_group;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            geometry_msgs::Pose target_pose1;
            ros::NodeHandle nh_; // Guarda el NodeHandle
    };
}
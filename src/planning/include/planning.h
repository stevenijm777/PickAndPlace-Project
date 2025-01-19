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
            void cartesianPath();
            void cartesianPath2();
            void resetValues();
            void addObjects();
            void makeTable();
            void goToInitialState();
            void makeBox(std::string blk_name, double *pose);
            void removeObjects();
            void goToPosition(double x, double y, double z);
            void goToJointArticulateState();
            void goRightPosition(double x, double y, double z);
            void controlGripper();
            void PickAndPlace();
            void OpenGripper();
            void CloseGripper();


        private:
            const std::string PLANNING_GROUP = "right_arm";

            moveit::planning_interface::MoveGroupInterface move_group;
            moveit::planning_interface::PlanningSceneInterface virtual_world;
            const robot_state::JointModelGroup* joint_model_group;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            geometry_msgs::Pose target_pose1;
    };
}
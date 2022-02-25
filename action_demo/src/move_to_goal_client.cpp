#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "action_demo/MoveToGoalAction.h"

typedef actionlib::SimpleActionClient<action_demo::MoveToGoalAction> Client;

// execute when the goal is done
void doneCb(const actionlib::SimpleClientGoalState& state)
{
    ROS_INFO("Yay! The copy goal operation is ok.");
}

// execute when the goal is active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// execute when receiving the feedback from server
void feedbackCb(const action_demo::MoveToGoalFeedbackConstPtr& feedback)
{
    ROS_INFO("i am ok .");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_goal_client");

    // create a client
    Client client("move_to_goal", true);

    // wait for server
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // create goal
    action_demo::MoveToGoalGoal goal;
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.pose.position.x = 1;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.position.z = 0;

    goal.target_pose.pose.orientation.w = 1;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    // send goal to server. set callback
    client.sendGoal(goal);

    client.waitForResult();

    ros::Duration(1).sleep();

    return 0;
}

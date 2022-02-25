#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "action_demo/MoveToGoalAction.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionServer<action_demo::MoveToGoalAction> Server;

// execute when receive a goal from client
void execute(const action_demo::MoveToGoalGoalConstPtr& goal, Server* as)
{
    ros::Rate rate(1);
    action_demo::MoveToGoalFeedback feedback;

    ROS_INFO("move to goal is working.");

    // 只是赋值
    geometry_msgs::PoseStamped current_pose;
    current_pose = goal->target_pose;
    feedback.base_position = current_pose;
    as->publishFeedback(feedback);
    rate.sleep();
    //获取目标值，并计算速度

    // done. return result
    action_demo::MoveToGoalResult result;
    result.total_dishes_cleaned = 10;
    as->setSucceeded(result);
    ROS_INFO("copy goal is done.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_goal_server");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    bool tf_ok = true;
    try {
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    } catch(tf::TransformException ex) {
        //ROS_ERROR("-------> %s", ex.what());
        tf_ok = false;
    }
    if(tf_ok) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom";

        pose_stamped.pose.position.x = transform.getOrigin().getX();
        pose_stamped.pose.position.y = transform.getOrigin().getY();
        pose_stamped.pose.position.z = transform.getOrigin().getZ();

        pose_stamped.pose.orientation.x = transform.getRotation().getX();
        pose_stamped.pose.orientation.y = transform.getRotation().getY();
        pose_stamped.pose.orientation.z = transform.getRotation().getZ();
        pose_stamped.pose.orientation.w = transform.getRotation().getW();

        tf::Matrix3x3 mat(tf::Quaternion(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                         pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w));

        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        ROS_INFO_STREAM("x: " << pose_stamped.pose.position.x);
        ROS_INFO_STREAM("y: " << pose_stamped.pose.position.y);
        ROS_INFO_STREAM("Theta: " << yaw);

    }
    // create a server
    Server server(nh, "move_to_goal", boost::bind(&execute, _1, &server), false);

    // start the server
    server.start();

    ros::spin();

    return 0;
}

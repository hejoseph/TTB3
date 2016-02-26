/*
 * SendGoals.cpp
 *
 *  Created on: Dec 7, 2013
 *      Author: roiyeho
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <kobuki_msgs/Sound.h>

ros::Publisher pub_atBase_;
ros::Publisher pub_nextClient_;
ros::Publisher pub_dockNow_;
ros::Publisher pub_Sound;
kobuki_msgs::Sound sound_;
std_msgs::Bool atbase_;
std_msgs::Int8 next_;
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void moveTo(const geometry_msgs::Twist::ConstPtr & coord);

int main(int argc, char** argv) {
    cout<<"beginning ...";
    ros::init(argc, argv, "move_to_point");

    next_.data=1;

    ros::NodeHandle nh;
    pub_atBase_ = nh.advertise<std_msgs::Bool>("at_base", 10);
    pub_nextClient_ = nh.advertise<std_msgs::Int8>("next_Client", 10);
    pub_Sound = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 10);
    pub_dockNow_ = nh.advertise<std_msgs::Int8>("dock_now", 10);

    ros::Subscriber sub_coffee = nh.subscribe("move_to", 1000, moveTo);



    ROS_INFO("omg");
     ros::spin();

    return 0;

}


void moveToSpecificPoint(double x, double y){
//    double x, y, theta;
    double theta;
    //nh.getParam("goal_x", x);
    ROS_INFO("The value of x is %f",x);
    //nh.getParam("goal_y", y);
    //nh.getParam("goal_theta", theta);

    // create the action client
    // true causes the client to spin its own thread
    MoveBaseClient ac("move_base", true);

    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(5));

    ROS_INFO("Connected to move base server");

    // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    //goal.target_pose.pose.position.x = 18.174;
    //goal.target_pose.pose.position.y = 28.876;
    //goal.target_pose.pose.orientation.w = 1;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    // Convert the Euler angle to quaternion
    double radians = theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);

    goal.target_pose.pose.orientation = qMsg;

    ROS_INFO("Sending goal to: x = %f, y = %f, theta = %f", x, y, theta);
    ac.sendGoal(goal);

    // Wait for the action to return
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("You have reached the goal!");
        
        if(x==1.0 && y==1.0){
            ROS_INFO("at Base !");
            atbase_.data=true;
            pub_atBase_.publish(atbase_);
            sound_.value = 4;
            pub_Sound.publish(sound_);
        }else if(x==4.62 && y==-0.68){
            ROS_INFO("reached the base !");
            ros::Duration(2).sleep();
            std_msgs::Int8 i;
            i.data = 1;
            pub_dockNow_.publish(i);
        }else{
            ROS_INFO("not comparable");
        }
    }else
        ROS_INFO("The base failed for some reason");

    // pub_nextClient_.publish(next_);
}

void moveTo(const geometry_msgs::Twist::ConstPtr & coord){
    ROS_INFO("Executing ... ");
    // Read x, y and angle parameters
    atbase_.data=false;
    pub_atBase_.publish(atbase_);
    moveToSpecificPoint(coord->linear.x,coord->linear.y);
    // ros::Duration(2).sleep();
    sound_.value=5;
    pub_Sound.publish(sound_);


}
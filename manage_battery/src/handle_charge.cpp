#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingActionGoal.h>
#include <kobuki_msgs/AutoDockingGoal.h>
#include <geometry_msgs/Twist.h>

int flag_state;
int flag_old_state;
ros::Publisher pub_goToPoint;
ros::Publisher pub_teleop;

void handleCharge(const std_msgs::Int8::ConstPtr & request);
void handleDock(const std_msgs::Int8::ConstPtr & request);
void dockAction();

int main(int argc, char** argv) {
	ros::init(argc, argv, "handle_charge");
	ros::NodeHandle nh;
	flag_state=1;
	flag_old_state=1;

	ros::Subscriber sub_charge = nh.subscribe<std_msgs::Int8>("service_availibility", 1000, handleCharge);
	ros::Subscriber sub_dockNow = nh.subscribe<std_msgs::Int8>("dock_now", 1000, handleDock);
	pub_goToPoint = nh.advertise<geometry_msgs::Twist>("move_to", 10);
	pub_teleop = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

	ros::spin();

}

bool state_switched(int a);
bool state_switched(int a){
	if(a!=flag_old_state){
		return true;
	} else {
		return false;
	}
}

void dockAction(){
	actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac("dock_drive_action", true);
	ac.waitForServer();
    kobuki_msgs::AutoDockingGoal goal;
    ac.sendGoal(goal);


    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
}

void handleCharge(const std_msgs::Int8::ConstPtr & request){
	flag_old_state = flag_state;
	flag_state = request->data;
	if(state_switched(flag_state)){
		geometry_msgs::Twist coord;
		if(flag_state==0){
			ROS_INFO("Going to base for charge");
			coord.linear.x = 0.10;
			coord.linear.y = 2.18;
			pub_goToPoint.publish(coord);
		}else {
			int i=0;
			coord.linear.x=-0.10;
			for(i=0;i<5;i++){
				ros::Duration(2).sleep();
				pub_teleop.publish(coord);
			}
		}
	}
}

void handleDock(const std_msgs::Int8::ConstPtr & request){
	ROS_INFO("Docking Now ...");
	dockAction();
}
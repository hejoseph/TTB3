
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <smart_battery_msgs/SmartBatteryStatus.h>

ros::Publisher pubService_;
std_msgs::Int8 flagCharge_;

//constantly checking the robot battery, and automatically make him go to dock station if needed
void processCharge(const smart_battery_msgs::SmartBatteryStatus::ConstPtr & request);

int main(int argc, char** argv) {
	ros::init(argc, argv, "netbook_battery");
	ros::NodeHandle nh;
	flagCharge_.data=1;
	ros::Subscriber sub_charge = nh.subscribe<smart_battery_msgs::SmartBatteryStatus>("laptop_charge", 1000, processCharge);

	pubService_ = nh.advertise<std_msgs::Int8>("service_availibility", 10);

	ros::spin();
}


void processCharge(const smart_battery_msgs::SmartBatteryStatus::ConstPtr & data){
	ROS_INFO("receiving percentage battery %d",data->percentage);
	ROS_INFO("receiving charge battery %f",data->charge);
	ROS_INFO("flag %d",flagCharge_.data);

	/*bad coordonnate*/
	if(data->percentage<70){
		ROS_INFO("Need Recharge ...");
		flagCharge_.data=0;
	}else if(data->percentage>=75){
		flagCharge_.data=1;
	}
	

	if(data->charge_state==1){
		ROS_INFO("Charging ...");
	} else {
		ROS_INFO("Not Charging ...");
	}
	pubService_.publish(flagCharge_);
}
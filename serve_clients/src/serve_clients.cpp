#include <ros/ros.h>
#include <custom_data/ClientArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

void serving_clients(const custom_data::ClientArray::ConstPtr & ca);
ros::Publisher pub_goToPoint;
ros::Publisher pub_changeState;


int main(int argc, char** argv) {
	ros::init(argc, argv, "serve_clients");
	ros::NodeHandle nh;

	//recupere la liste des clients/boissons validés
	ros::Subscriber sub_serve_clients = nh.subscribe<custom_data::ClientArray>("clients_toServe", 1000, serving_clients);
	pub_goToPoint = nh.advertise<geometry_msgs::Twist>("move_to", 10);
	pub_changeState = nh.advertise<std_msgs::Int8>("change_state", 10);

	ros::spin();
}

void serving_clients(const custom_data::ClientArray::ConstPtr & ca){
	ROS_INFO("Serving process ... \n\n");
	// ROS_INFO("name of the client %s", ca->clients[0].client_name.c_str());
	geometry_msgs::Twist twist;
	for(int i=0; i<ca->clients.size();i++){
		ROS_INFO("client %s", ca->clients[i].client_name.c_str());
		twist.linear.x = ca->clients[i].posx;
		twist.linear.y = ca->clients[i].posy;
		pub_goToPoint.publish(twist);
	}

	std_msgs::Int8 msg;
	msg.data = 0;
	pub_changeState.publish(msg);

}
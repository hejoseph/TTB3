#include <ros/ros.h>
#include <custom_data/Client.h>
#include <custom_data/ClientArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

custom_data::ClientArray ca_serving;

void serving_clients(const custom_data::ClientArray::ConstPtr & ca);
void processNext(const std_msgs::Int8::ConstPtr & msg);

ros::Publisher pub_goToPoint;
ros::Publisher pub_changeState;


int main(int argc, char** argv) {
	ros::init(argc, argv, "serve_clients");
	ros::NodeHandle nh;

	//recupere la liste des clients/boissons validés
	ros::Subscriber sub_serve_clients = nh.subscribe<custom_data::ClientArray>("clients_toServe", 1000, serving_clients);
	pub_goToPoint = nh.advertise<geometry_msgs::Twist>("move_to", 10);
	pub_changeState = nh.advertise<std_msgs::Int8>("change_state", 10);
	ros::Subscriber sub_nextClient = nh.subscribe<std_msgs::Int8>("next_Client", 1000, processNext);

	ros::spin();
}

void serving_clients(const custom_data::ClientArray::ConstPtr & ca){
	ROS_INFO("Serving process ... \n\n");
	// ROS_INFO("name of the client %s", ca->clients[0].client_name.c_str());
	ca_serving.clients = ca->clients;
	geometry_msgs::Twist twist;
	// for(int i=0; i<ca_serving.clients.size();i++){
	// 	ROS_INFO("client %s", ca_serving.clients[i].client_name.c_str());
		// twist.linear.x = ca_serving.clients[i].posx;
		// twist.linear.y = ca_serving.clients[i].posy;
		// pub_goToPoint.publish(twist);
	// }

	// ca_serving.clients.push_back(ca_.clients[0]);
	
	twist.linear.x = ca_serving.clients[0].posx;
	twist.linear.y = ca_serving.clients[0].posy;
	ca_serving.clients.erase(ca_serving.clients.begin());
	pub_goToPoint.publish(twist);

	

}

void processNext(const std_msgs::Int8::ConstPtr & msg){
	ROS_INFO("Next Client... \n\n");
	if(ca_serving.clients.size()>0){
		geometry_msgs::Twist twist;
		twist.linear.x = ca_serving.clients[0].posx;
		twist.linear.y = ca_serving.clients[0].posy;
		ca_serving.clients.erase(ca_serving.clients.begin());
		pub_goToPoint.publish(twist);
	} else {
		ROS_INFO("switch robot state... \n\n");
		std_msgs::Int8 msg;
		msg.data = 0;
		pub_changeState.publish(msg);
	}
}


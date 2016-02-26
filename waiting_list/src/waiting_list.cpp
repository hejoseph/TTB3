#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <custom_data/Client.h>
#include <custom_data/ClientArray.h>
#include <std_msgs/Int8.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/ButtonEvent.h>

const int MAX_ITEM=4;

using namespace std;
custom_data::ClientArray ca_;
custom_data::ClientArray ca_serving;
custom_data::Client c;
ros::Publisher pubClients_;
ros::Publisher pub_nextClient;
ros::Publisher pubClients_toBeServed;
ros::Publisher pub_robotState;
kobuki_msgs::Sound sound_;
ros::Publisher pub_Sound;

int flag_service;

// 0:stand_by, 1:loading_drinks, 2:serving_drinks, 3:charging
int robot_state;

int valid_pressed_;
int nb_item;
int total_item;
ros::Publisher pub_goToPoint;

void processService(const std_msgs::Int8::ConstPtr & msg);
void processRequest(const std_msgs::Int8::ConstPtr & request);
void processCommand(const custom_data::Client::ConstPtr & client);
// void valid_pressed(const std_msgs::Int8::ConstPtr & pressed);
void valid_pressed(const kobuki_msgs::ButtonEvent::ConstPtr & pressed);
void goToLoadDrink();
void goServing();
void manage_pressed();

void change_state(const std_msgs::Int8::ConstPtr & state);

int main(int argc, char** argv) {
	ros::init(argc, argv, "waiting_list");
	ros::NodeHandle nh;
	valid_pressed_ = 0;
	robot_state = 0;
	nb_item=0;
	total_item=0;
	flag_service=1;

	//send the list of clients waiting to the browser
	pubClients_ = nh.advertise<custom_data::ClientArray>("clients", 10);
	// ros::Rate loop_rate(0.05);

	//load clients list when page is requested
	ros::Subscriber sub_request = nh.subscribe<std_msgs::Int8>("request_page", 1000, processRequest);

	//save the name of the client and his position x y
	ros::Subscriber sub_command = nh.subscribe<custom_data::Client>("command_client", 1000, processCommand);

	ros::Subscriber sub_service = nh.subscribe<std_msgs::Int8>("service_availibility", 1000, processService);

	//listen when a drink is validated (button "Validate Drink" in the browser)
	ros::Subscriber sub_valid_pressed = nh.subscribe<kobuki_msgs::ButtonEvent>("/mobile_base/events/button", 1000, valid_pressed);
	// ros::Subscriber sub_valid_pressed = nh.subscribe<kobuki_msgs::ButtonEvent>("/mobile_base/events/button", 1000, valid_pressed);
	//send the validated drinks to be processed
	pubClients_toBeServed = nh.advertise<custom_data::ClientArray>("clients_toServe", 10);

	pub_nextClient = nh.advertise<std_msgs::Int8>("next_Client", 10);

	pub_Sound = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 10);


	pub_goToPoint = nh.advertise<geometry_msgs::Twist>("move_to", 10);
	pub_robotState = nh.advertise<std_msgs::Int8>("robot_state", 10);

	//listen robot state
	ros::Subscriber sub_changeState = nh.subscribe<std_msgs::Int8>("change_state", 1000, change_state);
	// ROS_INFO("omg");
	// c.client_name = "joseph";
	// ca_.clients.push_back(c);
	// while(ros::ok()){
	// 	ROS_INFO("publishing clients list to browser");
	// 	pubClients_.publish(ca_);
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
	ros::spin();
}

void processService(const std_msgs::Int8::ConstPtr & msg) {
	// ROS_INFO("Service State ...");
	flag_service=msg->data;
}


void change_state(const std_msgs::Int8::ConstPtr & msg) {
	ROS_INFO("changing state ...%d", msg->data);
	robot_state = msg->data;
	if (ca_.clients.size() > 0) {
		goToLoadDrink();
	}
}

void goToLoadDrink() {
	if (robot_state == 0) {
		//go load drink at the cafeteria
		geometry_msgs::Twist twist;
		twist.linear.x = 3.45;
		twist.linear.y = -1.26;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;
		pub_goToPoint.publish(twist);
		robot_state = 1;
		// sound_.value=5;
		// pub_Sound.publish(sound_);
	}
}

void processCommand(const custom_data::Client::ConstPtr & client) {
	if(flag_service==0){
		ROS_INFO("Service is not available");
	}else{
		ROS_INFO("we have an order ...\n\n");
		// ros::Duration(2).sleep();
		// sound_.value=5;
		// pub_Sound.publish(sound_);
		// ROS_INFO("started node ...%s",client->client_name.c_str());
		c.client_name = client->client_name;
		c.posx = client->posx;
		c.posy = client->posy;
		c.nb_item=client->nb_item;
		ROS_INFO(" nb : %d", c.nb_item);
		ca_.clients.push_back(c);
		pubClients_.publish(ca_);
		for (int i = 0; i < ca_.clients.size(); ++i) {
			const custom_data::Client &client = ca_.clients[i];
			ROS_INFO("Client %s", client.client_name.c_str());
		}


		goToLoadDrink();

		ROS_INFO("robot state : %d", robot_state);
	}

}

void goServing(){
	total_item=0;
	pubClients_toBeServed.publish(ca_serving);
	robot_state=2;
	ca_serving.clients.clear();
}

// void valid_pressed(const std_msgs::Int8::ConstPtr & pressed) {
// 	ROS_INFO("trying to validate a client's command ...\n");
// 	if (ca_.clients.size() > 0) {
// 		ROS_INFO("his command is being removed ...\n");
// 		if (valid_pressed_ < 4) {
// 			ca_serving.clients.push_back(ca_.clients[0]);
// 			ca_.clients.erase(ca_.clients.begin());
// 			valid_pressed_++;
// 			if (ca_.clients.size() == 0) {
// 				//publish
// 				pubClients_toBeServed.publish(ca_serving);
// 				ca_serving.clients.clear();
// 				valid_pressed_ = 0;
// 			}
// 		}

// 		if (valid_pressed_ == 4) {
// 			//publish
// 			pubClients_toBeServed.publish(ca_serving);
// 			ca_serving.clients.clear();
// 			valid_pressed_ = 0;
// 		}
// 		pubClients_.publish(ca_);
// 		// ROS_INFO("the client is ... %s",
// 	}
// }


void manage_pressed(){
	ROS_INFO("trying to validate a client's commands ...\n");
	if(robot_state==1){
		int test_total = 0;
		
		if (ca_.clients.size() > 0) {
			ROS_INFO("his command is being removed ...\n");
			ROS_INFO("debut");
			ROS_INFO("nb_total en cours : %d", total_item);
			if (valid_pressed_ < MAX_ITEM) {
				total_item+=ca_.clients[0].nb_item;
				if(total_item<=MAX_ITEM){
					ca_serving.clients.push_back(ca_.clients[0]);
					ca_.clients.erase(ca_.clients.begin());
					// total_item+=test_total;
					valid_pressed_++;
					if (ca_.clients.size() == 0) {
						//publish
						total_item=0;
						goServing();
						// pubClients_toBeServed.publish(ca_serving);
						// ca_serving.clients.clear();
						valid_pressed_ = 0;
					}
				}
				// }else{
				// 	total_item=0;
				// 	pubClients_toBeServed.publish(ca_serving);
				// 	ca_serving.clients.clear();
				// 	valid_pressed_ = 0;
				// }
			}
			ROS_INFO("middle");
			ROS_INFO("nb_total en cours : %d", total_item);
			// total_item+=ca_.clients[0].nb_item;
			if (total_item == MAX_ITEM) {
				//publish
				total_item=0;
				goServing();
				// pubClients_toBeServed.publish(ca_serving);
				// ca_serving.clients.clear();
				valid_pressed_ = 0;
			}


			if (ca_.clients.size() != 0) {
				test_total+=ca_.clients[0].nb_item;
				test_total+=total_item;
				ROS_INFO("liste pas vide");
				ROS_INFO("nb_total en cours : %d", total_item);
			}

			
			if (test_total > MAX_ITEM) {
				//publish
				// total_item=0;
				goServing();
				// pubClients_toBeServed.publish(ca_serving);
				// ca_serving.clients.clear();
				valid_pressed_ = 0;
			}
			ROS_INFO("fin");
			ROS_INFO("nb_total en cours : %d", total_item);
			pubClients_.publish(ca_);
			// ROS_INFO("the client is ... %s",
		}
	}
}

void valid_pressed(const kobuki_msgs::ButtonEvent::ConstPtr & pressed) {
	ROS_INFO("Button : %d, State %d", pressed->button, pressed->state);
	if(pressed->state==1){
		if(pressed->button==2){
			manage_pressed();
			// sound_.value=6;
		}
		if(pressed->button==1){
			goServing();
			// sound_.value=4;
		}
		if(pressed->button==0){
			std_msgs::Int8 msg;
			pub_nextClient.publish(msg);
			// sound_.value=5;
		}
			// pub_Sound.publish(sound_);
	}
}

void processRequest(const std_msgs::Int8::ConstPtr & request) {
	ROS_INFO("Requesting page ...");
	std_msgs::Int8 msg;
	msg.data = robot_state;
	pub_robotState.publish(msg);
	pubClients_.publish(ca_);
}
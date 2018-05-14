#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <time.h>


using namespace std;

// Globals
mavros_msgs::State current_state;
geometry_msgs::PoseStamped target_pose;

//setup arming code bool
mavros_msgs::CommandBool arm_status;
mavros_msgs::SetMode offb_set_mode;

//xbox joystick control
bool xbox_control = true;
float xbox_pose[6] = {0,0,0,0,0,0};
bool xbox_flag = false;

//Flight mode switch
int FlightMode = 1;

//ros time values
double time_stamp = 0;
double start_time = 0;

float k = 1.0;

//coefficients setup
void init();
void state_cb(const mavros_msgs::State::ConstPtr&);
void xbox_cb(const sensor_msgs::Joy&);
void pose_cb(const geometry_msgs::PoseStamped&);

//Service Client
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

// Main
int main(int argc, char **argv){

	init();
	ros::init(argc, argv, "figure_eight");
	ros::NodeHandle nh;

	// Subscribers
	ros::Subscriber state_subscriber = nh.subscribe("/mavros/state", 10, state_cb);
	ros::Subscriber joystick = nh.subscribe("/joy", 100, xbox_cb);
	ros::Subscriber quad_pose = nh.subscribe("/mavros/local_position/pose", 100, pose_cb);

	// Publishers
	ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	//begin timer
	//start_time = ros::Time::now().toSec();

	// Publish Rate
	ros::Rate rate(250.0);

	// wait for PixRacer
	while(ros::ok() && current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok())
	{

		if(xbox_flag)
		{
			set_mode_client.call(offb_set_mode);
			arming_client.call(arm_status);
			xbox_flag = false;
		}

		// Publish the desired position
		pose_publisher.publish(target_pose);
		time_stamp = ros::Time::now().toSec();

		//xbox_control
		if(FlightMode == 1)
		{
			target_pose.pose.position.x -= 0.01*xbox_pose[0];
			target_pose.pose.position.y += 0.01*xbox_pose[1];
			target_pose.pose.position.z += 0.002*xbox_pose[2];
		}

		//figure_eight
		if(FlightMode == 2)
		{
			target_pose.pose.position.x = sin(k*(time_stamp - start_time));
			target_pose.pose.position.y = sin(0.5*k*(time_stamp - start_time));
			target_pose.pose.position.z = 1.0;//+0.2*sin(0.25*k*(time_stamp - start_time));
		}

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

void init(){
	// Create and set target position
	target_pose.pose.position.x = 0; //temorary offset for flying two at once
	target_pose.pose.position.y = 0;
	target_pose.pose.position.z = 1;
	target_pose.pose.orientation.x = 0;
	target_pose.pose.orientation.y = 0;
	target_pose.pose.orientation.z = 0;
	target_pose.pose.orientation.w = 1;
}


// State callback
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

//xbox callback
void xbox_cb(const sensor_msgs::Joy& msg)
{
	if(msg.buttons[2] == 1)
	{
	//ARM = X
	offb_set_mode.request.custom_mode = "OFFBOARD";
	arm_status.request.value = true;
	set_mode_client.call(offb_set_mode);
        arming_client.call(arm_status);
	}
	if(msg.buttons[3] == 1)
	{
	//DISARM = B
	offb_set_mode.request.custom_mode = "STABILIZED";
	arm_status.request.value = false;
	set_mode_client.call(offb_set_mode);
        arming_client.call(arm_status);
	}
	if(msg.buttons[0] == 1)
	{
		xbox_flag = true;
		FlightMode = 1;
		ROS_INFO("Joystick Control");
	}
	if(msg.buttons[1] == 1)
	{
		xbox_flag = true;
		FlightMode = 2;
		start_time = ros::Time::now().toSec();
		ROS_INFO("Figure Eight");
  	}
	if(msg.buttons[5] == 1)
	{
		k += 0.5;
	}

	if(abs(msg.axes[3])>0.2) //X
		xbox_pose[0] = msg.axes[3];
	else
		xbox_pose[0] = 0;
	if(abs(msg.axes[4])>0.2) //Y
		xbox_pose[1] = msg.axes[4];
	else
		xbox_pose[1] = 0;
	if(abs(msg.axes[1])>0.2)  //Z
		xbox_pose[2] = msg.axes[1];
	else
		xbox_pose[2] = 0;
	if(abs(msg.axes[1])>0.2)  //YAW
		xbox_pose[3] = msg.axes[1];
	else
		xbox_pose[3] = 0;
}

void pose_cb(const geometry_msgs::PoseStamped& msg)
{

}

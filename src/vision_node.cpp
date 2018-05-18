#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "local_functions.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"

#include <stdio.h>

using namespace std;
using namespace cv;

//////// Global Variables //////////////////////
Point3_<float> stupidOffset(0, 0.0, 0);

Point3_<float> goal(-5,-5,-5);

int trajIterator = 0;

int currentMode = 1;

float Ts = 0.2;
float Th = 0.8; // time horizon : In multiples of Ts

vector<Point3f> currentTrajectory;
vector<float> currentOdometry;
vector<float> endState; 
bitset<4> escapeDirection(15);

geometry_msgs::PoseStamped lastCommandPose;

bool doEscape = 1;

ros::Publisher posePub;
ros::Publisher imgPub;

bool camInfo_isUpdated = 0;
bool odom_isUpdated = 0;

int flightMode = 1;

// Camera Model Parameters

int imgHeight;
int imgWidth;

Mat distortionVector(5,1,cv::DataType<double>::type);
Mat projectionMatrix(3,4,cv::DataType<double>::type);

// Safety Bounds

float x_bound[2] = {-0.6 , 5};
float y_bound[2] = {-3 , 3};
float z_bound[2] = {0 , 2};


////////////////////////////////////////////////

void depthCallback(const sensor_msgs::Image&);
void camInfoCallback(const sensor_msgs::CameraInfo&);
void odomCallback(const nav_msgs::Odometry&);
void timerCallback(const ros::TimerEvent&);
void flightModeCallback(const std_msgs::Int32&);
bool isBounded(geometry_msgs::PoseStamped&);
void setpoint_cb(const geometry_msgs::PoseStamped&);
void init();

int main(int argc, char **argv)
{
 	//cv::namedWindow( "imgdepth");// Create a window for display.
	//cv::startWindowThread();

	init();

  	ros::init(argc, argv, "vision_node");

  	ros::NodeHandle n;

	// Subscribers
	ros::Subscriber depthSub = n.subscribe("/zed/depth/depth_registered", 100, depthCallback);
	ros::Subscriber camInfoSub = n.subscribe("/zed/depth/camera_info", 100, camInfoCallback);
	ros::Subscriber odomSub = n.subscribe("/mavros/local_position/odom", 100, odomCallback);
	ros::Subscriber modeSub = n.subscribe("/flight_mode", 10, flightModeCallback);
	ros::Subscriber setpoint_sub = n.subscribe("/mavros/setpoint_position/local", 10, setpoint_cb);

	// Publishers
	posePub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local" ,100);
	imgPub = n.advertise<sensor_msgs::Image>("/labeled_image" ,100);

	ros::Timer timer = n.createTimer(ros::Duration(Ts), timerCallback);

  //ros::Rate loop_rate(10);
  
//  ros::spin();
  
  	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();

  //ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	//spinner.spin(); // spin() will not return until the node has been shutdown

	//timerCallback();
	
	//destroyAllWindows();
  return 0;
}

void init()
{
remove("/home/nvidia/ros_ws/src/vision_pkg/logs/flight_logs.txt");
}

bool isBounded(geometry_msgs::PoseStamped& point)
{
//float x_ref = setpoint.pose.position.x;
//float y_ref = setpoint.pose.position.y;
//float z_ref = setpoint.pose.position.z;
bool val;

val = 1;

if(point.pose.position.x < x_bound[0])
{
point.pose.position.x = x_bound[0];
val = 0;
}
if(point.pose.position.x > x_bound[1])
{
point.pose.position.x = x_bound[1];
val = 0;
}

if(point.pose.position.y < y_bound[0])
{
point.pose.position.y = y_bound[0];
val = 0;
}
if(point.pose.position.y > y_bound[1])
{
point.pose.position.y = y_bound[1];
val = 0;
}

if(point.pose.position.z < z_bound[0])
{
point.pose.position.z = z_bound[0];
val = 0;
}
if(point.pose.position.z > z_bound[1])
{
point.pose.position.z = z_bound[1];
val = 0;
}

return val;
}

void timerCallback(const ros::TimerEvent&)
{
	if(flightMode != 2)
	return;

	cout << " Timer Callback Called **************************************************************" << endl;

	if(trajIterator == currentTrajectory.size() || currentTrajectory.empty())
	{
	lastCommandPose.header.stamp = ros::Time::now();
	posePub.publish(lastCommandPose);

	cout << "End of Trajectory" << endl;
	//currentTrajectory.clear();
	return;
	}

	if(!currentTrajectory.empty())
	{
	cout << "Sending Next Waypoint" << currentTrajectory[trajIterator] << endl;
	geometry_msgs::PoseStamped commandPose;

	commandPose.pose.position.x = currentTrajectory[trajIterator].x - stupidOffset.x;
	commandPose.pose.position.y = currentTrajectory[trajIterator].y - stupidOffset.y;
	commandPose.pose.position.z = currentTrajectory[trajIterator].z - stupidOffset.z;

	commandPose.pose.orientation.x = 0;
	commandPose.pose.orientation.y = 0;
	commandPose.pose.orientation.z = 0;
	commandPose.pose.orientation.w = 1;

	commandPose.header.stamp = ros::Time::now();

	if(isBounded(commandPose))
	posePub.publish(commandPose);
	else
	{
	lastCommandPose.header.stamp = ros::Time::now();
        posePub.publish(lastCommandPose);
	}

	trajIterator += 1;
	}
	//getchar();
}


void depthCallback(const sensor_msgs::Image& msg)
{

	if(flightMode != 2)
	{
	string s = msg.encoding;
        ROS_INFO("%s", s.c_str());
	return;
	}

Mat imgDepth; //depth map

cv_bridge::CvImagePtr ptr;
ptr = cv_bridge::toCvCopy(msg,"32FC1");
imgDepth = ptr->image.clone();

//trajectory.push_back(Point3f(2,0,1));
//trajectory.push_back(Point3f(2.5,1.5,1));  
//trajectory.push_back(Point3f(3,0,1));
//trajectory.push_back(Point3f(3.5,-1.5,1));
//trajectory.push_back(Point3f(4,0,1));
//trajectory.push_back(Point3f(4.5,2,1));
//trajectory.push_back(Point3f(1,0,1));
//trajectory.push_back(Point3f(3,0,1));
//trajectory.push_back(Point3f(6,0,1));

ros::Time lasttime=ros::Time::now();

vector<int> areInCollision;

if(camInfo_isUpdated && odom_isUpdated)
{
cout << "Local Goal : " << goal << endl;
//getchar();
vector<Point3f> receivedTrajectory = waypoints_gen(endState, currentOdometry, currentMode, goal, 0.2, Ts, Th, escapeDirection, doEscape, projectionMatrix, distortionVector, imgDepth,  ptr->image);

	if(receivedTrajectory.empty())
	{
	cout << "Empty Trajectory Received" << endl;
	//getchar();
	//doReplan = 1;
	}
	else
	{
	cout << "Trajectory Received" << endl;
	currentTrajectory.insert(currentTrajectory.end(), receivedTrajectory.begin(), receivedTrajectory.end());
	
	//trajIterator = 0;
	//getchar();
	}	
	if(!currentTrajectory.empty())
	{
	cout << "currentTrajectory : " << currentTrajectory << endl;
	cout << "iterator : " << trajIterator << endl;
	}
	
	//getchar();
//cout << "Checking for Collisions ... " << endl;
//areInCollision = waypoints2collision(currentTrajectory, currentPose,0.5, projectionMatrix, distortionVector, imgDepth, ptr->image);
//cout << "Checked for Collisions ... " << endl;
 
//cout << " OUT " << endl;

//cout << "areInCollision : (";
//for (int i =0; i<3; i++)
//cout << areInCollision[i] << ",";
//cout << ")"<< endl;

ros::Time currenttime=ros::Time::now();

ros::Duration diff = currenttime - lasttime;
cout << " TIME TAKEN (s) : " << diff << endl;

imgPub.publish(ptr->toImageMsg());

	//cv::imshow( "imgdepth", imgDepth );                   // Show our image inside it.
	//waitKey(1);
 
//geometry_msgs::PoseStamped commandPose;
//commandPose.header.stamp = ros::Time::now();

//if(areInCollision[0] == 1 || areInCollision[1] == 1 || areInCollision[2] == 1)
//{
//commandPose.pose.position.x = 2;
//commandPose.pose.position.y = 2;
//commandPose.pose.position.z = 1;
//}
//else
//{
//commandPose.pose.position.x = 5;
//commandPose.pose.position.y = 0;
//commandPose.pose.position.z = 1;
//}

//commandPose.pose.orientation.x = 0;
//commandPose.pose.orientation.y = 0;
//commandPose.pose.orientation.z = 0;
//commandPose.pose.orientation.w = 1;
//}

//posePub.publish(commandPose);

//ROS_INFO("Number of Rows handling%i",imgDepth.size().height);
//ROS_INFO("Number of Cols %i",imgDepth.size().width);
//ROS_INFO("Number of Channels %i",imgDepth.channels());
//	string s = msg.encoding;
//	ROS_INFO("%s", s.c_str());
}
cout << "Exiting Depth Callback ... " << endl;
}

void camInfoCallback(const sensor_msgs::CameraInfo& msg)
{

imgHeight = msg.height; 												// The image dimensions with which the camera was calibrated
imgWidth = msg.width;

cout << "imgHeight : " << imgHeight << endl;
cout << "imgWidth : " << imgWidth << endl;
	
for(int i = 0; i < 3; i++) // Projection/camera matrix: By convention, this matrix specifies the intrinsic (camera) matrix of the processed (rectified) image
{
	for(int j = 0; j < 4; j++)
	{
	//cout << " i " << i << " j " << j << endl;
	projectionMatrix.at<double>(i,j) = msg.P[3*i + i + j];
 }
}

for(int i = 0; i < 5; i++) // Distortion Vector
	distortionVector.at<double>(i,0) = msg.D[i];


camInfo_isUpdated = 1;

/*
for(int i = 0; i < 3; i++) // Projection/camera matrix: For debugging
{
	for(int j = 0; j < 4; j++)
	cout << projectionMatrix.at<double>(i,j) << " ";
cout << endl;void flightModeCallback(const std_msgs::Int32&);
}
cout << endl << endl;

for(int i = 0; i < 5; i++) // Distortion vector: For debugging
	cout << distortionVector.at<double>(i,0) << " ";

cout << endl << endl;
*/
					
}

void odomCallback(const nav_msgs::Odometry& msg)
{
currentOdometry.clear();

currentOdometry.push_back(msg.pose.pose.position.x);
currentOdometry.push_back(msg.pose.pose.position.y);
currentOdometry.push_back(msg.pose.pose.position.z);

currentOdometry.push_back(msg.twist.twist.linear.x);
currentOdometry.push_back(msg.twist.twist.linear.y);
currentOdometry.push_back(msg.twist.twist.linear.z);

currentOdometry.push_back(msg.pose.pose.orientation.x);
currentOdometry.push_back(msg.pose.pose.orientation.y);
currentOdometry.push_back(msg.pose.pose.orientation.z);
currentOdometry.push_back(msg.pose.pose.orientation.w);

currentOdometry.push_back(msg.twist.twist.angular.x);
currentOdometry.push_back(msg.twist.twist.angular.y);
currentOdometry.push_back(msg.twist.twist.angular.z);

odom_isUpdated = 1;

cout << "Subscribed to Current Pose .... " << endl;

}

void flightModeCallback(const std_msgs::Int32& msg)
{
flightMode = msg.data;
}

void setpoint_cb(const geometry_msgs::PoseStamped& msg)
{
	lastCommandPose = msg;
}

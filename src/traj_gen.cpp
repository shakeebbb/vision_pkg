

#include "local_functions.h"

using namespace std;
using namespace cv;
using namespace Eigen;

vector<Point3f> waypoints_gen(vector<float>& endState, vector<float> odometry, int& currentMode, Point3_<float>& goal, float succRadius, float Ts, float Th, bitset<4>& escapeDirection, 
									bool& doEscape, Mat& projectionMatrix, Mat& distortionVector, Mat& imgDepth, Mat& imgRgb)
{
// Considers 3D double integrator model

// odometry : x, y, z, \dot(x), \dot(y), \dot(z), x, y, z, w, \dot(roll), \dot(pitch), \dot(yaw)
// states : x, \dot(x), y, \dot(y), z, \dot(z)
// pose : x, y, z, x, y, z, w

	// Preparing Log File ..........................................
	
	ofstream logFile;
	logFile.open ("/home/nvidia/ros_ws/src/vision_pkg/logs/flight_logs.txt", ios::app);
	const time_t ctt = time(0);
	logFile << asctime(localtime(&ctt)) << endl;

	// Setting Up Local Goal For The Trajectory Patch To Be Generated ...........................
	Point3_<float> globalGoal(4.0, 0, 1);	
		
	// Preparing Array For Carrying New Waypoints ..............................

	vector<Point3f> newWaypoints;
	newWaypoints.clear();
	
	Matrix<float, 3, 6> K_1;
	Matrix<float, 3, 6> K_2;
	Matrix<float, 3, 6> K_3;
	Matrix<float, 3, 6> K_4;
	Matrix<float, 3, 6> K; // current K / Mode
	
	// Preparing LQR Gains ...........................................
	
	K_1 <<	0.5178,    1.0307,   -0.0000,   -0.0000,    0.0000,    0.0000,
					0.0000,   -0.0000,    0.5178,    1.0307,    0.0000,    0.0000,
			    0.0000,    0.0000,   -0.0000,    0.0000,    0.5178,    1.0307;
			    
	K_2 <<  0.1679,    0.7861,    0.0000,    0.0000,    0.0000,    0.0000,
   			 -0.0000,    0.0000,    1.5073,    1.7428,   -0.0000,   -0.0000,
   			 -0.0000,    0.0000,    0.0000,   -0.0000,    1.5073,    1.7428;
	
	K_3 << 0.0540,    0.6321,    0.0000,    0.0000,    0.0000,   -0.0000,
   			-0.0000,    0.0000,    0.0540,    0.6321,   -0.0000,   -0.0000,
   			-0.0000,   -0.0000,   -0.0000,   -0.0000,    0.0540,    0.6321;
   			
  K_4 <<       2.4146,    2.3264,    0.0000,    0.0000,    0.0000,    0.0000,
   						-0.0000,   -0.0000,    2.4146,    2.3264,    0.0000,    0.0000,
   						-0.0000,    0.0000,    0.0000,    0.0000,    2.4146,    2.3264;
	
	// Preparing Safety Params .....................................
	float velInDrift = 1;
	float distIndrift = 0.5;
	float succRadiusInMode2 = 0.5;
	
	float safetyRadius = 0.5;	// (m) least distance between an obstacle and the vehicle
	
	float safetyTime = 0.8;	// (s) least time between last trajectory point in mode 3 and the collision : In multiples of Ts
	
	// ...............................................................
	
	cout << "Called waypoints_gen 2" << endl;
	
	vector<float> currentPose;
	currentPose.push_back(odometry[0]);
	currentPose.push_back(odometry[1]);
	currentPose.push_back(odometry[2]);
	currentPose.push_back(odometry[6]);
	currentPose.push_back(odometry[7]);
	currentPose.push_back(odometry[8]);
	currentPose.push_back(odometry[9]);

	cout << "Called waypoints_gen 3" << endl;	
	
	Matrix<float, 6, 1> initialState;
	
	if(endState.empty())
	{
	cout << "z" << endl;
	initialState << odometry[0], // x
									odometry[3], // \dot(x)
									odometry[1], // y
									odometry[4], // \dot(y)
									odometry[2], // z
									odometry[5]; // \dot(z)
	}
	else
	{
	initialState << endState[0], // x
									endState[1], // \dot(x)
									endState[2], // y
									endState[3], // \dot(y)
									endState[4], // z
									endState[5]; // \dot(z)
	}
									
	
	//float d = Vector3f(currentState(0,0) - goal.x, currentState(2,0) - goal.y, currentState(5,0) - goal.z).norm();
	//float d = abs(initialState(0,0) - globalGoal.x);
	
	cout << "Called waypoints_gen 4" << endl; 
	 
	if(abs(initialState(0,0) - globalGoal.x) < succRadius) 
	{
	cout << "M I S S I O N ... C O M P L E T E" << endl;
	
	logFile << "M I S S I O N ... C O M P L E T E" << endl;
	logFile.close();
	return newWaypoints;
	}
	
	cout << "currentMode : " << currentMode << endl;
	
	
	/////
	vector<float> end_state; // LQR solver returns the last state of the current patch of trajectory
	Matrix<float, 6, 1> goalState; // Local goal state for current trajectory patch
	vector<int> areInCollision; // Array of collision status of checked points from coming from LQR solver
	////		
	
	switch(currentMode)
		{
			/////////////////////////////////////////////////////////////////////
			case 0:
			{
			break;
			}
			///////////////////////////////////////////////////////////////////////////
			case 1:
			{
			cout << " Checking Feasibility in Mode 1" << endl;
			
			logFile << " # Mode 1 " << endl;
			
			goalState << 	globalGoal.x,
										0,
										globalGoal.y,
										0,
										globalGoal.z,
										0;
			K = K_1;
			
			//escapeDirection = 15;
			break;
			}
			
			////////////////////////////////////////////////////////////////
			case 2:
			{	
			cout << " Checking Feasibility in Mode 2" << endl;
			
			logFile << " # Mode 2 " << endl;
			
			logFile << "Initial State = " << initialState << endl;
			
			if(goal == Point3_<float>(-1, -1, -1))
				{
				cout << " Stuckkkk " << endl;
				
				logFile << " Stuckkkk ... Sending Empty Trajectory " << endl;
				logFile.close();
				
				return newWaypoints;
				}
				
			//if(escapeDirection == bitset<4>(0))
			//	{
			//	escapeDirection = 15;
			//	}

			
			if(abs(initialState(0,0) - goal.x) < succRadiusInMode2) 
				{
				currentMode = 3;
				doEscape = 0;
				
				cout << "LOCAL ... M I S S I O N ... C O M P L E T E" << endl;
				
				logFile << " Finished Generating Avoid Trajectory ... Going To Mode 3 with doEscape = " << doEscape << ", Avoid Goal = " << goal << endl;
				logFile.close();
				
				return newWaypoints;
				}
			
			goalState << 	goal.x,
										0,
										goal.y,
										0,
										goal.z,
										0;
			K = K_4;
			break;
			}
				
			////////////////////////////////////////////////////////////////////////////////////////
			case 3:
			{
			cout << " Checking Feasibility in Mode 3" << endl;
			logFile << " # Mode 3 " << endl;
			
				if(abs(odometry[0] - goal.x) < succRadiusInMode2)
				{
				doEscape = 1;
				
				logFile << " Vehicle Close To Local Avoid Goal ... Retaining Mode 3 with doEscape = " << doEscape << ", Avoid Goal = " << goal << endl;
				}
			
			goalState << 	globalGoal.x,
										0,
										goal.y,
										0,
										goal.z,
										0;
			K = K_1;
			
			cout << "goalState : "<< goalState << endl;
			
			logFile << " New Goal : " << goalState << endl;
							//getchar();
							
			//escapeDirection = 15;				
			break;
			}
			///////////////////////////////////////////////////////////////////
			default:
			{
			break;
			}
			///////////////////////////////////////////////////////////////////
		}
			newWaypoints = lqr_solver(initialState, goalState, Th, safetyTime, Ts, K, end_state);
			
			cout << " Got New Waypoints From LQR Solver: " << newWaypoints << endl;
			
			logFile << " Got New Waypoints From LQR Solver: " << newWaypoints << endl;
			logFile << " Checking For Collisions .......... " << endl;
			
			areInCollision = waypoints2collision(newWaypoints, currentPose, safetyRadius, projectionMatrix, distortionVector, imgDepth, imgRgb);
			
			int collisionWaypointIndex = -1;
			for(int i=0; i<areInCollision.size(); i++)
			{
				if(areInCollision[i] == 1)
				{
				collisionWaypointIndex = i;
				break;
				}
			}
			
			cout << "Current Mode : " << currentMode << endl;
			
			if(collisionWaypointIndex != -1 && !doEscape)
			{
			logFile << "Point in Collision : " << newWaypoints[collisionWaypointIndex] << endl;
			logFile << "Restting Waypoints ... " << endl;
			newWaypoints.clear();
			}
			
			else if(collisionWaypointIndex != -1  && doEscape)
			{
				cout << "Point in Collision : " << newWaypoints[collisionWaypointIndex] << endl;
				
				logFile << "Point in Collision : " << newWaypoints[collisionWaypointIndex] << endl;
				logFile << "Finding Escape in Directions : " << escapeDirection << endl;
				
				cout << "FINDING ESCAPE " << endl;
				//getchar();
				bitset<4> direction = escapeDirection;
				goal = find_escape(direction, newWaypoints[collisionWaypointIndex], currentPose, safetyRadius, projectionMatrix, distortionVector, imgDepth, imgRgb);
				//escapeDirection = escapeDirection & direction.flip();
				
				logFile << "Found Escape in Direction : " << direction << endl;
				
				cout << "GOAL CHANGED : " << goal << endl;
				
				logFile << "Goal Changed : " << goal << endl;
				logFile << "Will Enter Mode 2 Next Time To Build Avoid Trajectory ............... " << endl;
				
				currentMode = 2;
				newWaypoints.clear();
				//getchar();
			}
			else
			{
			newWaypoints.erase(newWaypoints.begin() + Th/Ts, newWaypoints.end());
			endState = end_state;
			cout << " New End State : " << endState[0] << endl;
			cout << " Trimmed Waypoints: " << newWaypoints << endl;
			
			logFile << " Trimmed Waypoints: " << newWaypoints << endl;
			logFile << " New End State: [ " << endState[0] << ", " << endState[1] << ", " << endState[2] << ", " << endState[3] << ", " 
																			<< endState[4] << ", " << endState[5] << "]" << endl;
			}

	cout << "Returning newWaypoints : " << newWaypoints << endl;
	logFile.close();
	return newWaypoints;
}

vector<Point3f> lqr_solver(MatrixXf initialState, MatrixXf finalState, float Th, float safetyTime, float Ts, MatrixXf K, vector<float>& endState)
{

// Considers 3D double integrator model
// Th : Time Horizon
// Ts : Sampling Time 

cout << "Called lqr_solver" << endl;
cout << " Initial State : " << initialState << endl;
cout << " Final State : " << finalState << endl;

vector<Point3f> newWaypoints;

int N = floor( (Th + safetyTime) / Ts ) + 1; // number of waypoints : generate trajectory for an additional safetyTime seconds
int n = 6; // number of states
int m = 3; // number of inputs

Matrix<float, 6, 6> A;
Matrix<float, 6, 3> B;

A << 	   1,  0.2,    0,    0,    0,    0,
         0,    1,    0,    0,    0,    0,
    	   0,    0,    1,  0.2,    0,    0,
    	   0,    0,    0,    1,    0,    0,
     	 	 0,    0,    0,    0,    1,  0.2,
     		 0,    0,    0,    0,    0,    1;
			
B << 	0.02, 	   0,   	  0,
      	0.2,     0,   	  0,
       	 0,	  0.02,     0,
       	 0,	   0.2,     0,
       	 0,	     0,  	0.02,
       	 0,	     0, 	  0.2;

MatrixXf X(N, n);
MatrixXf U(N, m);

//for(int i=0; i<n; i++)
//X(1,n) = initialState(i);

cout << "Called lqr_solver 1" << endl;

X.row(0) = initialState.transpose();
//newWaypoints.push_back(Point3_<float>(X.row(0)(0), X.row(0)(2), X.row(0)(4)));

cout << "Called lqr_solver 2" << endl;

for (int i=1; i<N; i++)
	{
	cout << "Called lqr_solver 2 a" << endl;
	X.row(i) = ((A - B * K) * X.row(i-1).transpose() + B * K * finalState).transpose();
	
	cout << X.row(i) << endl;
	
	cout << "Called lqr_solver 2 b" << endl;
	
	newWaypoints.push_back(Point3_<float>(X.row(i)(0), X.row(i)(2), X.row(i)(4)));
	
	cout << "Called lqr_solver 2 c" << endl;
	
	cout << (- K * (X.row(i-1).transpose() - finalState)).transpose() << endl;
	
	U.row(i-1) = (-K * (X.row(i-1).transpose() - finalState) ).transpose();
	}
	
	cout << "Called lqr_solver 2 d" << endl;
 	U.row(N-1) = (-K * (X.row(N-1).transpose() - finalState) ).transpose();
 	
cout << "Called lqr_solver 3" << endl;

cout << " Waypoints Generated " << " : " << newWaypoints << endl;
cout << "State : " << X << endl;
	endState.clear();
	endState.push_back(X(Th/Ts,0));
	endState.push_back(X(Th/Ts,1));
	endState.push_back(X(Th/Ts,2));
	endState.push_back(X(Th/Ts,3));
	endState.push_back(X(Th/Ts,4));
	endState.push_back(X(Th/Ts,5));


cout << " Returning Waypoints from LQR Solver" << endl;

cout << " Returning New End State From LQR : " << endState[0] << endl;
return newWaypoints;
}



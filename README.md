# vision_pkg
Code For Thesis

C++ Functions Called from vision_node

traj_gen : Lqr solver function is for waypoints generation for a certain time horizon.
		   Waypoints gen checks the trajectory received from lqr solver for collisions and controls mode changes
						 
waypoints2collision : waypoints2collision transforms each waypoint received from traj_gen, transforms it to camera coordinates and send to cspace2collision for collision checking
		      findescape finds the escape in all directions represented by the binary 4 digit and returns the escape point or (-1,-1,-1) if all directions are blocked 

cspace2collision : forms a rectangle around and infront of the given point according to the safetyRadius and checks each point in that for collision or occlusions. Right now occlusions are treated as no collision while forming trajectories

PARAMETERS:

vision_node : stupidOffset subtracts the steady offset if any while sending each waypoint to mavros
	      Ts is the sampling time of the discrete controller/system
	      Th is the time horizon for the trajectory generation
	      Subscribers and publishers topics

traj_gen :    logFile location to save the flight logs
	      globalGoal to set the final destination of the robot
	      lqr gains K1, K2, K3, K4 for different controller modes . K1 and K4 are currently used.
						  
waypoints2collision : camOffset saves camera offset from the robot origin to ensure safety radius is at right location around the robot
		      dplane, dprojections in findescape to set the location of the rectagular sheild
		      

REFERENCES:

Ahmad, Shakeeb. "High-Performance Testbed for Vision-Aided Autonomous Navigation for Quadrotor UAVs in Cluttered Environments." (2018). https://digitalrepository.unm.edu/ece_etds/416 
										  
				




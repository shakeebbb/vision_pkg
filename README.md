# vision_pkg
Code For Thesis

figure_eight.launch : launches mavros, figure_eight_node, vicon_remap
  figure_eight_node : quadrotor hovers at initial point in A mode while going for figure eight from initial point when B is pressed
  vicon_remap : Remaps transform msg from /vicon/quad3/quad3 to pose stamped msg /mavros/mocap/pose
  
  Need to roslaunch vicon_bridge vicon.launch on the pc (source  ~/vicon_ws/devel/setup.bash)
  
vision.launch : launches vision_rempap, drone_pose_node, vision_node
  vision_remap : Remaps odometry msg from /zed/odom to pose stamped msg /mavros/mocap/pose
  drone_pose_node : Quadrotor hover at initial point in A mode while doing nothing in B mode
  vision_node : Does nothing in A mode, takes over in B mode for vision navigation

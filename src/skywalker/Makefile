control:
	@rosservice call /enable_motors "enable: true"
	@rosrun teleop_twist_keyboard teleop_twist_keyboard.py

indoor:
	@roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch

outdoor:
	@roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch

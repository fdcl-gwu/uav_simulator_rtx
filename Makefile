PACKAGE=uav_gazebo
f=uav_rover

.PHONY: launch

launch:
	@roslaunch $(PACKAGE) $(f).launch

launch-rover:
	@roslaunch $(PACKAGE) rover.launch

simple:
	@roslaunch $(PACKAGE) simple_world.launch

uav:
	@cd ./scripts && python3 uav_main.py

rover:
	@cd ./scripts && python3 rover_main.py
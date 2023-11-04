PACKAGE=uav_gazebo
f=uav_rover

.PHONY: launch

launch:
	@roslaunch $(PACKAGE) $(f).launch

rover:
	@roslaunch $(PACKAGE) rover.launch

simple:
	@roslaunch $(PACKAGE) simple_world.launch

uav-old:
	@cd ./scripts && python3 main.py

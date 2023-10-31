PACKAGE=uav_gazebo
LAUNCH_FILE=simple_world.launch

.PHONY: launch

launch:
	@roslaunch $(PACKAGE) $(LAUNCH_FILE)

uav:
	@cd ./scripts && while true; do python3 main.py; sleep 0.1; done

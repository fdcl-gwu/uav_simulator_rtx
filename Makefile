PACKAGE=uav_gazebo
f=uav_rover

.PHONY: launch

## launch commands

launch:
	@roslaunch $(PACKAGE) $(f).launch

launch-rover:
	@roslaunch $(PACKAGE) rover.launch

simple:
	@roslaunch $(PACKAGE) simple_world.launch

killall:
	@killall -9 gzserver gzclient python3
	@killall rosmaster roscore rosout

## GUI/Control commands

uav:
	@cd ./scripts && python3 uav_main.py

rover:
	@cd ./scripts && python3 rover_main.py



# node commands

nlist:
	@rosnode list

ninfo:
	@rosnode info $(n)

node_graph:
	@rosrun rqt_graph rqt_graph


## topic commands

tlist:
	@rostopic list

tinfo:
	@rostopic info $(t)

ttype:
	@rostopic type $(t)

techo:
	@rostopic echo $(t) --noarr


## service commands

slist:
	@rosservice list

ssend:
	@rosservice	call $(s) $(args)

reset_world:
	@rosservice call /gazebo/reset_world

reset_sim:
	@rosservice call /gazebo/reset_simulation


# ROS tools

rviz:
	@rviz

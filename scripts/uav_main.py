#!/usr/bin/env python

from utils.rover import rover, reset_uav
from utils.gui import thread_gui
from utils.thread_imu import thread_imu
from utils.thread_gps import thread_gps
from utils.thread_control import thread_control
from utils.thread_log import thread_log

import numpy as np
import rospy
import std_msgs
import threading
import time

IMU_TOPIC_NAME = 'uav_imu'
CONTROL_TOPIC_NAME = 'uav_fm'
GPS_TOPIC_NAME = 'uav_pos'


def run_uav():

    rospy.init_node('uav', anonymous=True)
    reset_uav()

    # Create threads
    t1 = threading.Thread(target=thread_control, 
                          args=(CONTROL_TOPIC_NAME,))
    t2 = threading.Thread(target=thread_imu,
                          args=(IMU_TOPIC_NAME,))
    t3 = threading.Thread(target=thread_gps,
                          args=(GPS_TOPIC_NAME,))
    t4 = threading.Thread(target=thread_gui)
    t5 = threading.Thread(target=thread_log)
    
    # Start threads.
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()

    # Wait until all threads close.
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()


if __name__ == '__main__':
    run_uav()

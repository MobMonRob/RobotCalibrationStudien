#!/usr/bin/env python

import sys
import time

from ur_msgs.srv import *
from ur_msgs.msg import IOStates
from std_msgs.msg import String

import rospy

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input


class SensorGetter:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self, service = '/ur_hardware_interface/io_states'):
        
        print('Getting subscriber')
        #rospy.wait_for_service(service)

        self.subscriber = rospy.Subscriber(service, String, self.callback)
        self.buttonState = False


    def callback(data):
        state = data.digital_in_states[0].state
        rospy.loginfo(state)
        buttonState = state

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/ur_hardware_interface/io_states", IOStates, self.callback)
        rospy.spin()

    def getSensorState(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/ur_hardware_interface/io_states", IOStates, self.callback)
        time.sleep(0.005)
        return self.buttonState

if __name__ == "__main__":
    client = SensorGetter()
    #message = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
    #print(message)
    client.listener()


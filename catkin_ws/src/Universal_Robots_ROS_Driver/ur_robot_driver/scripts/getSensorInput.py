#!/usr/bin/env python

import sys

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


def callback(data):
    state = data.digital_in_states[0].state
    rospy.loginfo(state)
    #print(data.data)
    if state == True:
        print('Was true')
    else:
        print('Was false')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/ur_hardware_interface/io_states", IOStates, callback)
    rospy.spin()


if __name__ == "__main__":
    #client = SensorGetter()
    #message = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
    #print(message)
    listener()


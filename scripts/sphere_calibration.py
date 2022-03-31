#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from asyncio.windows_events import NULL
from ctypes import create_unicode_buffer

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from getSensorInput import SensorGetter

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



class Calibration(object):

    def __init__(self):
        super(Calibration, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def plan_cartesian_path(self, x, y, z, oX, oY, oZ, oW):
        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.orientation.x = oX
        wpose.orientation.y = oY
        wpose.orientation.z = oZ
        wpose.orientation.w = oW
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )
        return plan, fraction

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def MoveToSphere(self, args, vector, offset, stepWidth, sensorGetter): #vector = zu fahrende richtung
        currentX = args[1] - offset * vector[0]
        currentY = args[2] - offset * vector[1]
        currentZ = args[3] - offset * vector[2]
        startPosPath = self.plan_cartesian_path(currentX, currentY, currentZ, 0.0, 0.0, 0.0, 0.0)
        self.execute_plan(startPosPath)

        stopX = currentX + 2 * offset * vector[0]
        stopY = currentX + 2 * offset * vector[1]
        stopZ = currentX + 2 * offset * vector[2]

        while((not sensorGetter.getSensorState) or self.MaximumReached(currentX, stopX, vector[0]) or self.MaximumReached(currentY, stopY, vector[1]) or self.MaximumReached(currentZ, stopZ, vector[2])):
            currentX += vector[0] * stepWidth
            currentY += vector[1] * stepWidth
            currentZ += vector[2] * stepWidth
            plan = self.plan_cartesian_path(currentX, currentY, currentZ, 0.0, 0.0, 0.0, 0.0)
            self.execute_plan(plan)

        result = self.move_group.get_current_pose().pose
        self.execute_plan(startPosPath)
        self.execute_plan(NULL) #TODO HomePos
        return result
    
    def MaximumReached(current, stop, vector):
        if(vector > 0):
            return current >= stop
        else: return current <= stop

def main():
    try:
        args = sys.argv
        calibration = Calibration()
        sensorGetter = SensorGetter()
        result = []

        sensorAttachementLength = 0.16
        safetyOffset = 0.05
        offset = sensorAttachementLength + safetyOffset
        stepWidth = 0.0005

        result.append(calibration.MoveToSphere(args, [1, 0, 0], offset, stepWidth, sensorGetter))
        result.append(calibration.MoveToSphere(args, [0, 1, 0], offset, stepWidth, sensorGetter))
        result.append(calibration.MoveToSphere(args, [0, -1, 0], offset, stepWidth, sensorGetter))
        result.append(calibration.MoveToSphere(args, [0, 0, -1], offset, stepWidth, sensorGetter))

        print("Pose1:\n")
        print(result[0] + "\n")
        print("Pose2:\n")
        print(result[1] + "\n")
        print("Pose3:\n")
        print(result[2] + "\n")
        print("Pose4:\n")
        print(result[3] + "\n")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
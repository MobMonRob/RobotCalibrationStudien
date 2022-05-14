#!/usr/bin/env python

from time import sleep
from sphere_calibration import JointBasedMove
import sys
import rospy
import moveit_commander
import moveit_msgs.msg


class LaserCalibration:
    def __init__(self):
        self.jointMover = JointBasedMove()

        #VALUES FOR A LASER-SCREEN DISTANCE OF 6.17m SCREEN IS 798x607mm
        self.baseHomePos = [-168.1, -107.1, 96, 189, 82, 0]
        #self.joint1plus =   -164.6, -109.75, 93.35, 191.5,  85.4, 
        #                    -171.5, -104.5,  98.65, 186.45, 78.7
        self.offsets = [3.4,2.6,2.65,2.5,3.3,45]
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("laser_calibration", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.moveGroup = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


    def move(self, joint, direction):
        newVals = self.baseHomePos
        newVals[joint] += (direction * self.offsets[joint])
        self.jointMover.go_to_joint_states(newVals, self.moveGroup)

    def rotateEndeffector(self, degree):
        newVals = self.baseHomePos
        newVals[5] = degree
        print(newVals)
        self.jointMover.go_to_joint_states(newVals, self.moveGroup)

    def testRotateEndeffector(self):
        for i in range(0,90,5):
            self.rotateEndeffector(-i)
            raw_input("...")
            self.rotateEndeffector(0)
            raw_input("...")


def main():
    try:
        calibration = LaserCalibration()

        calibration.testRotateEndeffector()

        #for joint in range(0,len(self.baseHomePos)):
        #    calibration.move(joint, 1)
        #    calibration.move(joint, -1)

        

    except rospy.ROSInterruptException:
        print("Test1")
        return
    except KeyboardInterrupt:
        print("Test2")
        return

if __name__ == "__main__":
    main()
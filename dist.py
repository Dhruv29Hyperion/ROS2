#!/usr/bin/python3

import math
import rospy
from turtlesim.msg import Pose


class TurtleDistanceMeasure:
    
    @staticmethod
    def DISTANCE_POSE(pose1,pose2): 
        return math.sqrt((pose1.x-pose2.x)**2+(pose1.y-pose2.y)**2)

    def __init__(self):
        self._pose_turtle1 = Pose()
        self._pose_turtle2 = Pose()

    # save turtle1 pose
    def callback_turtle1(self,msg): 
        self._pose_turtle1 = msg

    # save turtle2 pose
    def callback_turtle2(self,msg): 
        self._pose_turtle2 = msg

    def get_distance(self): 
        return TurtleDistanceMeasure.DISTANCE_POSE(self._pose_turtle1,self._pose_turtle2)


if __name__=="__main__":
    rospy.init_node("node_turtle_measure")

    turtle_distance = TurtleDistanceMeasure()
    sub_pose_1 = rospy.Subscriber("pose1",Pose,callback=turtle_distance.callback_turtle1)
    sub_pose_2 = rospy.Subscriber("pose2",Pose,callback=turtle_distance.callback_turtle2)

    rate = rospy.Rate(20)
    while(not rospy.is_shutdown()): 
        distance = turtle_distance.get_distance()
        rospy.loginfo(f"distance: {distance}")
        rate.sleep()

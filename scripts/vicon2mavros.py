#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from functools import partial

class Vicon2Mavros:
    def __init__(self):
        rospy.init_node('vicon2mavros')
        for i in range(1, 11):
            rospy.Subscriber(f"/vicon/p230_{i}/p230_{i}", TransformStamped, partial(self.callback, i=i))
        self.publishers = [rospy.Publisher(f"/uav{i}/mavros/vision_pose/pose", PoseStamped, queue_size=10) for i in range(1, 11)]
        rospy.spin()


    def callback(self, msg: TransformStamped, i):
        pose = PoseStamped()

        pose.header = msg.header
        
        pose.pose.position.x = msg.transform.translation.x
        pose.pose.position.y = msg.transform.translation.y
        pose.pose.position.z = msg.transform.translation.z

        pose.pose.orientation.x = msg.transform.rotation.x
        pose.pose.orientation.z = msg.transform.rotation.z
        pose.pose.orientation.y = msg.transform.rotation.y
        pose.pose.orientation.w = msg.transform.rotation.w

        self.publishers[i-1].publish(pose)


if __name__ == "__main__":
    Vicon2Mavros()
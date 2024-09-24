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
        self.pose = [PoseStamped() for i in range(1, 11)]

        rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.spin()

    def callback(self, msg: TransformStamped, i):
        self.pose[i - 1].header = msg.header
        self.pose[i - 1].header.frame_id = 'map'
        
        self.pose[i - 1].pose.position.x = msg.transform.translation.x
        self.pose[i - 1].pose.position.y = msg.transform.translation.y
        self.pose[i - 1].pose.position.z = msg.transform.translation.z

        self.pose[i - 1].pose.orientation.x = msg.transform.rotation.x
        self.pose[i - 1].pose.orientation.z = msg.transform.rotation.z
        self.pose[i - 1].pose.orientation.y = msg.transform.rotation.y
        self.pose[i - 1].pose.orientation.w = msg.transform.rotation.w

    def timer_callback(self, event):
        for i in range(1, 11):
            self.publishers[i - 1].publish(self.pose[i - 1])

if __name__ == "__main__":
    Vicon2Mavros()
#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation as R

def callback(msg: PoseStamped):
    quaternion = msg.pose.orientation

    euler = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_euler('xyz', degrees=True)

    print(f'Roll: {euler[0]:.2f}, Pitch: {euler[1]:.2f}, Yaw: {euler[2]:.2f}')

rospy.init_node('quaternion2euler')
rospy.Subscriber('/uav1/mavros/vision_pose/pose', PoseStamped, callback)
rospy.spin()

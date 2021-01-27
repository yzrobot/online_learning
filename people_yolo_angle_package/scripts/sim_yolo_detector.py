#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose
import names
import math
from std_msgs.msg import Header
import datetime
import tf


class people_watchtower():
    def __init__(self):
        self.angles = PoseArray()

        self.pub2 = rospy.Publisher(
            '/sim_yolo_angles', PoseArray, queue_size=1)

        self.angles.header = Header()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.angles.header.stamp.secs = now.secs
            self.angles.header.stamp.nsecs = now.nsecs
            self.angles.header.frame_id = '/base_link'

            temp_angles = []

            for angle in range(0,180):

                pose = Pose()
                position_y_rad = angle * math.pi / 180
        	    # pose.position.x = 1
                pose.position.y = 0
                q = tf.transformations.quaternion_from_euler(0,0,position_y_rad)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                temp_angles.append(pose)

            self.angles.poses = temp_angles

            self.pub2.publish(self.angles)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('sim_yolo_angle_detector')
    rate=rospy.Rate(2)
    people_watchtower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

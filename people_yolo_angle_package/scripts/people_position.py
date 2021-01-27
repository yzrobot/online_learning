#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from people_msgs.msg import People
from people_msgs.msg import Person
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from geometry_msgs.msg import PoseArray, Pose
import names
import math
import tf


class people_watchtower():
    def __init__(self):
        self.people = People()
        self.angles = PoseArray()
        self.vis_angles = PoseArray()
        self.measurements = PositionMeasurementArray()

        self.pub = rospy.Publisher(
            '/people_yolo_detector', People, queue_size=10)

        self.pub2 = rospy.Publisher(
            '/people_yolo_angles', PoseArray, queue_size=1)

        self.pub3 = rospy.Publisher(
            '/vis_yolo_angles', PoseArray, queue_size=1)

        self.pub4 = rospy.Publisher(
            '/people_yolo_measurements', PositionMeasurementArray, queue_size=1)

        self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes",
                                    BoundingBoxes, self.callback)

    def callback(self, data):

        self.people.header = data.header
        self.people.header.frame_id = "camera_rgb_optical_frame"

        self.angles.header = data.header

        self.vis_angles.header = data.header

        self.measurements.header = data.header
        self.measurements.header.frame_id = "camera_rgb_optical_frame"

        person_list = data.bounding_boxes

        temp = []
        temp_angles = []
        vis_angles = []
        pma = []

        for ppl in person_list:
            person = Person()
            pose = Pose()
            vis_pose = Pose()
            pm = PositionMeasurement()

            y = (ppl.xmax + ppl.xmin) / 2

            # # Position relative to world (center of camera is 90 degrees)
            # position_y = -0.090625*(y - 640) + 60

            # Position relative to robot (center of camera is 0 degrees)
            position_y = (y - 320) * 29 / 320

            position_y_rad = position_y * math.pi / 180

            # pose.position.x = 1
            pose.position.y = position_y_rad
            pose.orientation.w = 1

            person.position.y = position_y
            person.reliability = ppl.probability
            person.name = ppl.Class + "_" + names.get_first_name()

            q = tf.transformations.quaternion_from_euler(0,0,position_y_rad)
            vis_pose.orientation.x = q[0]
            vis_pose.orientation.y = q[1]
            vis_pose.orientation.z = q[2]
            vis_pose.orientation.w = q[3]

            pm.pos.y = position_y
            pm.reliability = ppl.probability
            pm.object_id = "-1" 

            temp_angles.append(pose)
            temp.append(person)
            vis_angles.append(vis_pose)
            pma.append(pm)

        self.people.people = temp
        self.angles.poses = temp_angles
        self.vis_angles.poses = vis_angles
        self.measurements.people = pma

        self.pub.publish(self.people)
        self.pub2.publish(self.angles)
        self.pub3.publish(self.vis_angles)
        self.pub4.publish(self.measurements)


if __name__ == '__main__':
    rospy.init_node('people_yolo_angle_detector', anonymous=True)
    people_watchtower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

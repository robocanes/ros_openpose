#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# echo.py: sample script to print ros message to terminal
# Author: Ravi Joshi
# Date: 2020/03/02

# import modules
import rospy
from ros_openpose.msg import Frame


def callback(msg):
    # text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    # rospy.loginfo('%s\n' % text)


    for person in msg.persons: 
      left_elbow = person.bodyParts[6]
      left_wrist = person.bodyParts[7]
      right_elbow = person.bodyParts[3]
      right_wrist = person.bodyParts[4]

      # m = (y2 - y1) / (x2 - x1)
      # b = y1 - (m * x1)
      # minX = min(x1, x2) used for limiting our lower bound
      # maxX = max(x1, x2) used for limiting our upper bound
      flat_x_l = False
      flat_y_l = False
      flat_x_r = False
      flat_y_r = False
      # if (left_elbow.pixel.x-left_wrist.pixel.x):
      #   flat_x_l = True
      if (left_wrist.pixel.x-left_elbow.pixel.x) == 0:
        flat_y_l = True
        slope_left = 99999999999999999999999999999999999999999999999999999999999999999999999999999999999
      # if (right_elbow.pixel.x-right_wrist.pixel.x):
      #   flat_x_r = True
      if (right_elbow.pixel.x-right_wrist.pixel.x) == 0:
        flat_y_r = True
        slope_right = 9999999999999999999999999999999999999999999999999999999999999999999999999999999999999

      if not flat_y_l:
        slope_left = (left_wrist.pixel.y-left_elbow.pixel.y)/(left_wrist.pixel.x-left_elbow.pixel.x)
      left_b = left_wrist.pixel.y - (slope_left * left_wrist.pixel.x)
      if not flat_y_r:
        slope_right = (right_wrist.pixel.y-right_elbow.pixel.y)/(right_wrist.pixel.x-right_elbow.pixel.x)
      right_b = right_wrist.pixel.y - (slope_right * right_wrist.pixel.x)

      points_left = []
      points_right = []
      x = 0
      while x < 640:
        line_point_l = [x, slope_left*x + left_b]
        line_point_r = [x, slope_left*x + left_b]
        points_left.append(line_point_l)
        points_right.append(line_point_r)
        x=x+1
      
      print(points_left[3])

def main():

    try:
        rospy.init_node("echo", anonymous=False)
    except rospy.exceptions.ROSException as e:
        print("Node has already been initialized, do nothing")
    # rospy.init_node('echo', anonymous=False)

    # read the parameter from ROS parameter server
    frame_topic = rospy.get_param('~pub_topic')

    rospy.Subscriber(frame_topic, Frame, callback)

    rospy.spin()


if __name__ == '__main__':
    main()

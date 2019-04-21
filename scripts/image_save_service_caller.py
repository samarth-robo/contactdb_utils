#!/usr/bin/python
import rospy
import cv2
import numpy as np
from std_srvs.srv import Empty

if __name__ == '__main__':
  rospy.init_node('image_save_service_caller', anonymous=True)

  kinect_srv_name = 'kinect_image_saver/save'
  rospy.loginfo('Waiting for service {:s} ...'.format(kinect_srv_name))
  rospy.wait_for_service(kinect_srv_name)
  thermal_srv_name = 'thermal_image_saver/save'
  rospy.loginfo('Waiting for service {:s} ...'.format(thermal_srv_name))
  rospy.wait_for_service(thermal_srv_name)

  kinect_srv = rospy.ServiceProxy(kinect_srv_name, Empty)
  thermal_srv = rospy.ServiceProxy(thermal_srv_name, Empty)

  im = np.zeros((50, 50, 3), dtype=np.uint8)
  cv2.imshow('press spacebar', im)

  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    choice = cv2.waitKey(5)
    if choice == ord(' '):
      # rospy.loginfo('Calling services')
      kinect_srv()
      thermal_srv()

    rate.sleep()

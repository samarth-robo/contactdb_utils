#!/home/samarth/libraries/anaconda2/bin/python
"""
ROS node that subscribes to RGB and Thermal image topics and writes them to disk
when you press spacebar
"""
import rospy
from sensor_msgs.msg import Image
import message_filters
import cv2
import os.path as osp
from cv_bridge import CvBridge
import argparse
import sys

time_diff_thresh = 0.01

class ImagePairWriter:
  def __init__(self, output_dir):
    self.bridge = CvBridge()
    self.output_dir = osp.expanduser(output_dir)
    self.counter = 0
    self.skip_prob = 0.75
    self.time_diff_thresh = time_diff_thresh

  def callback(self, rgb_image, thermal_image):
    # check for time difference of set returned by approximate sync policy
    time_diff = abs(rgb_image.header.stamp.to_sec() - thermal_image.header.stamp.to_sec())
    if time_diff > self.time_diff_thresh:
      rospy.loginfo('Time difference {:f} is too large, dropping!'.format(time_diff))
      return

    # get images
    try:
      if '16' in rgb_image.encoding:
        rgb_image.encoding = 'mono16'
      im_rgb = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='mono8')
      im_thr = self.bridge.imgmsg_to_cv2(thermal_image, desired_encoding='mono8')
    except rospy.ROSException as e:
      print 'Error in cv_bridge', e
      return

    cv2.imshow('rgb', im_rgb)
    cv2.imshow('thermal', im_thr)
    choice = cv2.waitKey(7)

    if choice != ord(' '):
      return

    # save images
    rgb_im_name = osp.join(self.output_dir,
        '{:05d}_rgb.jpg'.format(self.counter))
    thr_im_name = osp.join(self.output_dir,
        '{:05d}_thermal.jpg'.format(self.counter))
    rgb_written = cv2.imwrite(rgb_im_name, im_rgb)
    if not rgb_written:
      rospy.logerr('Could not write {:s}'.format(rgb_im_name))
    thr_written = cv2.imwrite(thr_im_name, im_thr)
    if not thr_written:
      rospy.logerr('Could not write {:s}'.format(thr_im_name))

    if rgb_written and thr_written:
      rospy.loginfo('Image pair {:d} written'.format(self.counter))
      self.counter += 1

def listener(output_dir):
  ipw = ImagePairWriter(output_dir)

  rospy.init_node('image_pair_writer', anonymous=True)
  rospy.loginfo('Node started')

  rgb_sub = message_filters.Subscriber('/deepgrasp/kinect2/qhd/image_color',
                                       Image)
  thermal_sub = message_filters.Subscriber('/deepgrasp/boson/image_raw', Image)

  ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, thermal_sub],
    queue_size=5, slop=time_diff_thresh)
  ts.registerCallback(ipw.callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    cv2.destroyAllWindows()

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--output_dir', required=True, help='Output directory')
  myargv = rospy.myargv(argv=sys.argv)
  args = parser.parse_args(myargv[1:])

  listener(args.output_dir)
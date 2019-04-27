#!/usr/bin/env python
import rospy
import sys
from camera_info_manager import CameraInfoManager, CameraInfoMissingError
from sensor_msgs.msg import CameraInfo

class CamInfoChanger(object):
  def __init__(self, old_format=False):
    self.old_format = old_format
    self.pub = rospy.Publisher('cam_info_out', CameraInfo, queue_size=1)
    if self.old_format:
      rospy.loginfo('CamInfoChanger is set to old format (960x540 image)')

    self.manager = CameraInfoManager(cname='boson',
      url='package://contactdb_utils/calibrations/boson.yaml')
    try:
      self.manager.loadCameraInfo()
    except IOError as e:
      rospy.logerr('Could not load calibration: {:s}'.format(e))

  def callback(self, c_in):
    c_out = CameraInfo()
    try:
      c_out = self.manager.getCameraInfo()
    except CameraInfoMissingError as e:
      rospy.logerr(e)

    if self.old_format:
      # scaling factor
      h_scaling = float(c_in.height) / c_out.height
      w_scaling = float(c_in.width) / c_out.width

      c_out.height = c_in.height
      c_out.width  = c_in.width

      # scale the intrinsics matrix K
      c_out.K[0] *= w_scaling
      c_out.K[2] *= w_scaling
      c_out.K[4] *= h_scaling
      c_out.K[5] *= h_scaling

      # scale the projection matrix P
      c_out.P[0] *= w_scaling
      c_out.P[2] *= w_scaling
      c_out.P[5] *= h_scaling
      c_out.P[6] *= h_scaling

    c_out.header = c_in.header
    self.pub.publish(c_out)


if __name__ == '__main__':
  rospy.init_node('change_camera_info')
  argv = rospy.myargv(sys.argv)
  remap = {'true': True, 'false': False}
  old_format = remap[argv[1]] if len(argv) > 1 else False
  cc = CamInfoChanger(old_format)
  rospy.Subscriber('cam_info_in', CameraInfo, cc.callback)
  rospy.spin()

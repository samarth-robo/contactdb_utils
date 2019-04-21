import numpy as np
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from camera_info_manager import saveCalibration, loadCalibrationFile
import rospy

class Camera:
  def __init__(self, camera_name, im_size, frame_id, dist_model='plumb_bob'):
    self.camera_name = camera_name
    self.im_size = im_size
    self.dist_model = dist_model

    # create Header
    self.header = Header()
    self.header.frame_id = frame_id

    self.K = np.eye(3, dtype=np.float64)
    n_dist_coeffs = 5 if self.dist_model == 'plumb_bob' else 8
    self.D = np.zeros((1, n_dist_coeffs), dtype=np.float64)
    self.R = np.eye(3, dtype=np.float64)
    self.P = np.eye(3, dtype=np.float64)

  def init_from_yaml(self, init_filename):
    try:
      ci = loadCalibrationFile(filename=init_filename, cname=self.camera_name)
    except IOError:
      print 'WARNING: {:s} camera info could not be loaded from {:s}'.\
        format(self.camera_name, init_filename)

    self.im_size = (ci.width, ci.height)
    self.dist_model = ci.distortion_model
    self.header = ci.header
    self.K = np.asarray(ci.K).reshape((3, 3))
    self.D = np.asarray(ci.D)
    self.D = self.D[np.newaxis, :]
    self.R = np.asarray(ci.R).reshape((3, 3))
    self.P = np.asarray(ci.P).reshape((3, 4))
    self.P = self.P[:3, :3]

  def write_to_yaml(self, out_filename):
    # create CameraInfo message
    ci = CameraInfo()
    # ci.header = self.header
    # ci.height = self.im_size[1]
    # ci.width = self.im_size[0]
    # ci.distortion_model = self.dist_model
    # ci.D[:] = self.D.flatten()
    # ci.K[:] = self.K.flatten()
    # ci.R[:] = self.R.flatten()
    # ci.P[:9] = self.P.flatten()

    # write to YAML
    done = saveCalibration(new_info=ci, url=out_filename, cname=self.camera_name)
    if done:
      print '{:s} written'.format(out_filename)
    else:
      print 'WARNING: {:s} camera info not saved'.format(self.camera_name)

  def print_info(self):
    print '*** Information for camera {:s}'.format(self.camera_name)
    print 'Calibration matrix'
    print self.K.flatten()
    print 'Distortion coefficients'
    print self.D.flatten()
    print 'Rotation matrix'
    print self.R.flatten()
    print 'Projection matrix'
    print self.P.flatten()
"""
Script to get clicks on chessboard corners by user and calibrate cameras
"""
from camera import Camera
import numpy as np
import cv2
import argparse
import os.path as osp
import glob
import pickle
import sys

def mouse_callback(event, x, y, flags, param):
  if event is not cv2.EVENT_FLAG_LBUTTON:
    return
  # print 'Got point ({:d}, {:d})'.format(x, y)
  im, points, window_name = param
  points.append([x, y])
  im = cv2.circle(im, (x, y), 3, (0, 0, 255), -1)
  show_image(window_name, im)

def show_image(window_name, im):
  cv2.namedWindow(window_name, flags=cv2.WINDOW_NORMAL)
  if window_name == 'kinect':
    cv2.moveWindow(window_name, 10, 500)
    cv2.resizeWindow(window_name, 604, 500)
  elif window_name == 'thermal':
    cv2.moveWindow(window_name, 650, 500)
    cv2.resizeWindow(window_name, 604, 500)
  cv2.imshow(window_name, im)

class Calibrator:
  def __init__(self, data_dir, output_dir, corners_filename=None, cb_height=6,
               cb_width=9, cb_dim=0.0236, dist_model='rational_polynomial'):
    self.data_dir = osp.expanduser(data_dir)
    self.cb_height = cb_height
    self.cb_width = cb_width
    self.cb_dim = cb_dim
    self.dist_model = dist_model
    output_dir = osp.expanduser(output_dir)
    self.thermal_yaml_filename = osp.join(output_dir, 'thermal_camera.yaml')
    self.kinect_yaml_filename = osp.join(output_dir, 'kinect.yaml')
    self.stereo_filename = osp.join(output_dir, 'stereo.pkl')

    # create ideal 3D object points
    self.objp = np.zeros((self.cb_width * self.cb_height, 3), dtype=np.float32)
    self.objp[:, :2] = self.cb_dim * np.mgrid[0:self.cb_width, 0:self.cb_height].T.reshape(-1, 2)

    fs = sorted(glob.glob(osp.join(self.data_dir, '*kinect.jpg')))
    print 'Found {:d} Kinect images.'.format(len(fs))
    self.kinect_im_names = [osp.join(self.data_dir, f) for f in fs]
    self.thermal_im_names = [osp.join(self.data_dir, f.replace('kinect', 'thermal')) for f in fs]

    im = cv2.imread(self.thermal_im_names[0])
    self.thermal_im_size = (im.shape[1], im.shape[0])
    im = cv2.imread(self.kinect_im_names[0])
    self.kinect_im_size = (im.shape[1], im.shape[0])

    if corners_filename is None:
      # need to extract corners
      self.kinect_image_points = {}
      self.thr_image_points = {}
    else:
      # load extracted corners from pickle file
      with open(osp.expanduser(corners_filename), 'rb') as f:
        self.kinect_image_points, self.thr_image_points = pickle.load(f)
      print 'Loaded {:d} corner sets from {:s}'.format(len(self.kinect_image_points),
                                                           corners_filename)

    # cameras
    self.tcam = Camera('thermal_camera', self.thermal_im_size, self.dist_model)
    self.kcam = Camera('kinect', self.kinect_im_size, self.dist_model)

    # initialization Kinect from YAML file
    self.kcam.init_from_yaml(osp.expanduser(
      '~/catkin_ws/src/contactdb_utils/calibrations/kinect.yaml'))
    self.kcam.print_info()

  def show_points(self, im, points, window_name):
    im_show = im.copy()
    if len(im_show.shape) == 2:  # grayscale
      im_show = cv2.cvtColor(im_show, cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(im_show, (self.cb_width, self.cb_height), points, True)
    show_image(window_name, im_show)

  def get_corners(self):
    term_criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100,
                     sys.float_info.epsilon)

    for idx, (kinect_filename, thermal_filename) in \
        enumerate(zip(self.kinect_im_names, self.thermal_im_names)):
      im_kinect = cv2.imread(kinect_filename)
      if im_kinect is None:
        print 'Could not read {:s}'.format(kinect_filename)
        continue
      im_thermal = cv2.imread(thermal_filename, cv2.IMREAD_GRAYSCALE)
      if im_thermal is None:
        print 'Could not read {:s}'.format(thermal_filename)
        continue

      choice = 0
      # keep looping same image till choice is 'y', 's' or 'q'
      while (choice != ord('y')) and (choice != ord('q')) and (choice != ord('s')):
        flags = cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_ADAPTIVE_THRESH + \
                 cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_NORMALIZE_IMAGE
        if kinect_filename in self.kinect_image_points:
          kinect_found = True
          rc = self.kinect_image_points[kinect_filename]
          print 'Corners already exist in pickle'
        else:
          kinect_found, rc =\
            cv2.findChessboardCorners(cv2.cvtColor(im_kinect, cv2.COLOR_BGR2GRAY),
                                      (self.cb_width, self.cb_height),
                                      flags=flags)
          rc = np.squeeze(rc)
        if not kinect_found:
          # get clicks from user
          print '[Kinect {:d}/{:d}] Click corners in order and then press any key. ' \
                's=skip, q=quit.'.format(idx+1, len(self.kinect_im_names))
          rp = []
          im_kinect_show = im_kinect.copy()
          show_image('kinect', im_kinect_show)
          show_image('thermal', im_thermal)
          cv2.setMouseCallback('kinect', mouse_callback, (im_kinect_show, rp,
                                                          'kinect'))
          choice = cv2.waitKey(-1)
          cv2.destroyAllWindows()
          cv2.waitKey(1)
          if (choice == ord('s')) or (choice == ord('q')):
            print 'Skipping this image'
            break

          if len(rp) != (self.cb_height * self.cb_width):
            print 'Only {:d} corners clicked, need {:d} x {:d}'.\
              format(len(rp), self.cb_width, self.cb_height)
            break

          # construct special np arrays for corners
          rc = np.zeros((len(rp), 2), dtype=np.float32)
          for ii, r in enumerate(rp):
            rc[ii, 0] = r[0]
            rc[ii, 1] = r[1]

        if kinect_filename not in self.kinect_image_points:
          # get sub-pixel corner values
          ds = [np.hypot(rc[i+1][0]-rc[i][0], rc[i+1][1]-rc[i][1])
            for i in xrange(len(rc)-1)]
          subpix_d = int(min(ds)/2.0)
          cv2.cornerSubPix(cv2.cvtColor(im_kinect, cv2.COLOR_BGR2GRAY), rc,
                           (subpix_d, subpix_d), (-1, -1), term_criteria)

        self.show_points(im_kinect, rc, 'kinect')
        print 'See subpixel corners. OK? (y/n/s/q)'
        choice = cv2.waitKey(-1)
        if choice == ord('s'):
          print 'Skipping this image'
          continue
        elif choice == ord('n'):
          print 'Repeating this image'
          continue
        elif choice == ord('q'):
          print 'Quitting'
          break

        if thermal_filename in self.thr_image_points:
          thr_found = True
          tc = self.thr_image_points[thermal_filename]
          print 'Corners already exist in pickle'
        else:
          thr_found, tc = cv2.findChessboardCorners(255-im_thermal,
            (self.cb_width, self.cb_height), flags=flags)

        if not thr_found:
          # get clicks from user
          print '[Thermal {:d}/{:d}] Click corners in order (from red to blue) and then ' \
                'press any key. s=skip, q=quit.'.format(idx, len(self.thermal_im_names))
          tp = []
          im_thermal_show = im_thermal.copy()
          show_image('thermal', im_thermal_show)
          # show_image('kinect', im_kinect)
          cv2.setMouseCallback('thermal', mouse_callback, (im_thermal_show, tp, 'thermal'))
          choice = cv2.waitKey(-1)
          if (choice == ord('s')) or (choice == ord('q')):
            print 'Skipping this image'
            break

          if len(tp) != (self.cb_height * self.cb_width):
            print 'Only {:d} corners clicked, need {:d} x {:d}'. \
              format(len(tp), self.cb_width, self.cb_height)
            break

          # construct special np arrays for corners
          tc = np.zeros((len(tp), 2), dtype=np.float32)
          for ii, t in enumerate(tp):
            tc[ii, 0] = t[0]
            tc[ii, 1] = t[1]

        if thermal_filename not in self.thermal_im_names:
          # get sub-pixel corner values
          ds = [np.hypot(tc[i+1][0]-tc[i][0], tc[i+1][1]-tc[i][1])
            for i in xrange(len(tc)-1)]
          subpix_d = int(min(ds)/2.0)
          cv2.cornerSubPix(im_thermal, tc, (subpix_d, subpix_d), (-1, -1),
            term_criteria)

        self.show_points(im_thermal, tc, 'thermal')
        print 'See subpixel corners. OK? (y/n/s/q)'
        choice = cv2.waitKey(-1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        if choice == ord('s'):
          print 'Skipping this image'
          continue
        elif choice == ord('n'):
          print 'Repeating this image'
          continue
        elif choice == ord('q'):
          print 'Quitting'
          break

        # store state
        self.kinect_image_points[kinect_filename] = rc
        self.thr_image_points[thermal_filename] = tc
        with open(osp.join(self.data_dir, 'corners.pk'), 'wb') as f:
          pickle.dump((self.kinect_image_points, self.thr_image_points), f)
          print 'Saved corners'

      if choice == ord('q'):
        break

  def calibrate_individual_cameras(self):
    # flags = cv2.CALIB_FIX_PRINCIPAL_POINT
    flags = 0
    if self.dist_model == 'rational_polynomial':
      flags += cv2.CALIB_RATIONAL_MODEL

    term_criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200,
    sys.float_info.epsilon)

    # calibrate thermal camera
    image_points = []
    object_points = []
    for _,v in self.thr_image_points.items():
      image_points.append(v)
      object_points.append(self.objp)

    print 'Calibrating thermal camera...'
    calib_out = cv2.calibrateCamera(object_points, image_points,
      self.thermal_im_size, self.tcam.K, self.tcam.D, rvecs=None, tvecs=None,
      flags=flags, criteria=term_criteria)
    thermal_reproj_error, self.tcam.K, self.tcam.D, _, _ = \
      calib_out
    self.tcam.print_info()
    print 'Reprojection error = {:f}'.format(thermal_reproj_error)
    # self.tcam.write_to_yaml(self.thermal_yaml_filename)

  def register_cameras(self):
    k_image_points = []
    t_image_points = []
    object_points = []
    assert len(self.kinect_image_points) == len(self.thr_image_points)
    for kinect_key, v in self.kinect_image_points.items():
      thermal_key = kinect_key.replace('kinect', 'thermal')
      if thermal_key in self.thr_image_points:
        k_image_points.append(v)
        t_image_points.append(self.thr_image_points[thermal_key])
        object_points.append(self.objp)

    print 'Calibrating two cameras together...'
    flags = cv2.CALIB_FIX_INTRINSIC
    if self.dist_model == 'rational_polynomial':
      flags += cv2.CALIB_RATIONAL_MODEL

    term_criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200,
                     sys.float_info.epsilon)

    # calibrate
    reg_reproj_error, self.tcam.K, self.tcam.D, self.kcam.K, self.kcam.D,\
      self.R, self.T, self.E, self.F = cv2.stereoCalibrate(object_points,
      t_image_points, k_image_points,
      self.tcam.K, self.tcam.D, self.kcam.K, self.kcam.D,
      self.thermal_im_size, R=None, T=None, E=None, F=None,
      flags=flags, criteria=term_criteria)

    #  rectify
    # self.tcam.R, self.kcam.R, self.tcam.P, self.kcam.P, self.Q, _, _ =\
    #   cv2.stereoRectify(self.tcam.K, self.tcam.D, self.kcam.K, self.kcam.D,
    #     self.im_size, self.R, self.T, alpha=0.5)

    self.tcam.print_info()
    self.kcam.print_info()
    print '*** Extrinsics'
    print 'R'
    print self.R
    print 'T'
    print self.T
    print 'E'
    print self.E
    print 'F'
    print self.F
    print 'Re-projection error = {:f}'.format(reg_reproj_error)
    # self.tcam.write_to_yaml(self.thermal_yaml_filename)
    # self.kcam.write_to_yaml(self.kinect_yaml_filename)
    with open(self.stereo_filename, 'w') as f:
      pickle.dump({'R': self.R, 'T': self.T, 'E': self.E, 'F': self.F}, f)
    print '{:s} written'.format(self.stereo_filename)

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dir', required=True, help='Directory holding Kinect and thermal images')
  parser.add_argument('--corners', help='Pickle file containing pre-extracted corners')
  parser.add_argument('--out_dir', required=True, help='Output directory for YAML files')
  args = parser.parse_args()

  ct = Calibrator(args.data_dir, args.out_dir, args.corners)
  ct.get_corners()
  ct.calibrate_individual_cameras()
  ct.register_cameras()

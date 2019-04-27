"""
This script is intended to be used on the output of play_bag_video.launch
It registers the RGB images to the thermal camera frame, and fills holes in
the depth images.
It assumes that the depth images are already registered to the thermal camera
frame (set registered:=true while running play_bag_video.launch)
"""
import numpy as np
import cv2
import os
import pickle
from depth_proc import fill_holes
from scipy.ndimage.interpolation import map_coordinates
import camera_info_manager as cinfo_manager
import matplotlib.pyplot as plt
import argparse
osp = os.path

class RegisterVideo(object):
  def __init__(self, data_dir):
    self.data_dir = osp.join(osp.expanduser(data_dir))

    # RGB camera info
    cinfo_filename = cinfo_manager.getPackageFileName(
      'package://contactdb_utils/calibrations/kinect.yaml')
    kinect_cinfo = cinfo_manager.loadCalibrationFile(cinfo_filename, 'kinect')
    self.rgb_im_size = (kinect_cinfo.width, kinect_cinfo.height)
    self.rgb_K = np.reshape(kinect_cinfo.K, (3, 3))

    # 3D transform between RGB and thermal
    stereo_filename = osp.join('..', 'calibrations', 'stereo.pkl')
    with open(stereo_filename, 'rb') as f:
      d = pickle.load(f)
      T = np.eye(4)
      T[:3, :3] = d['R']
      T[:3, 3:] = d['T']
    self.T_rgb_t = T


  @staticmethod
  def _load_thermal_cinfo(thermal_dir):
    cinfo_filename = osp.join(thermal_dir, 'camera_info.txt')
    thermal_cinfo = np.loadtxt(cinfo_filename)
    im_size = (int(thermal_cinfo[-2]), int(thermal_cinfo[-1]))
    K = np.eye(3)
    K[0, 0] = thermal_cinfo[0]
    K[1, 1] = thermal_cinfo[1]
    K[0, 2] = thermal_cinfo[2]
    K[1, 2] = thermal_cinfo[3]
    return im_size, K


  def register_images(self, object_name, session_name, show=False):
    base_dir = osp.join(self.data_dir, session_name, object_name)

    # thermal camera info
    thermal_dir = osp.join(base_dir, 'thermal_images')
    thermal_im_size, thermal_K = self._load_thermal_cinfo(thermal_dir)

    video_dir = osp.join(base_dir, 'sync_images')
    _, _, filenames = os.walk(video_dir).next()
    rgb_filenames = [osp.join(video_dir, fn) for fn in filenames if 'rgb' in fn]
    depth_filenames = [osp.join(video_dir, fn) for fn in filenames if 'depth' in fn]
    assert len(depth_filenames) == len(rgb_filenames)

    done = True
    for rgb_filename, depth_filename in \
      zip(sorted(rgb_filenames), sorted(depth_filenames)):
      u_thermal, v_thermal = np.meshgrid(np.arange(thermal_im_size[0]),
          np.arange(thermal_im_size[1]))
      uv_thermal = np.vstack((
        u_thermal.ravel(), v_thermal.ravel(), np.ones(u_thermal.size))).astype(int)
      xyz_thermal = np.dot(np.linalg.inv(thermal_K), uv_thermal)
      im_depth = cv2.imread(depth_filename, -1)
      im_depth = fill_holes(im_depth)
      d_thermal = im_depth.ravel() / 1000.0
      xyz_thermal *= d_thermal[np.newaxis, :]
      xyz_rgb = np.dot(self.T_rgb_t,
        np.vstack((xyz_thermal, np.ones(xyz_thermal.shape[1]))))

      uv_rgb = np.dot(self.rgb_K, xyz_rgb[:3])
      uv_rgb = uv_rgb[:2] / uv_rgb[2]

      # interpolate
      im_rgb = cv2.imread(rgb_filename, cv2.IMREAD_COLOR).astype(np.float) / 255
      b = map_coordinates(im_rgb[:, :, 0], uv_rgb[::-1], order=3).\
        reshape((thermal_im_size[1], thermal_im_size[0]))
      b = np.clip(b, 0, 1)
      g = map_coordinates(im_rgb[:, :, 1], uv_rgb[::-1], order=3).\
        reshape((thermal_im_size[1], thermal_im_size[0]))
      g = np.clip(g, 0, 1)
      r = map_coordinates(im_rgb[:, :, 2], uv_rgb[::-1], order=3).\
        reshape((thermal_im_size[1], thermal_im_size[0]))
      r = np.clip(r, 0, 1)
      im_thermal = np.stack((r, g, b), axis=2)

      if show:
        plt.figure()
        plt.imshow(im_thermal)
        plt.show()

      # save
      rgb_filename, ext = rgb_filename.split('.')
      rgb_filename = '{:s}_registered.{:s}'.format(rgb_filename, ext)
      im_thermal *= 255
      done = done and cv2.imwrite(rgb_filename, im_thermal[:, :, ::-1].astype(np.uint8))
      if not done:
        print('### WARN: Could not write {:s}'.format(rgb_filename))

      depth_filename, ext = depth_filename.split('.')
      depth_filename = '{:s}_registered.{:s}'.format(depth_filename, ext)
      done = done and cv2.imwrite(depth_filename, im_depth.astype(np.uint16))
      if not done:
        print('### WARN: Could not write {:s}'.format(depth_filename))
    return done


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dir',
    default=osp.join('..', 'data', 'contactdb_data'))
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--object_name', required=True)
  args = parser.parse_args()

  rr = RegisterVideo(data_dir=osp.expanduser(args.data_dir))
  rr.register_images(object_name=args.object_name, session_name=args.session_name)

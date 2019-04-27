"""
This script registers the RGB images to the thermal images
"""
import numpy as np
import cv2
import os
import pickle
from depth_proc import fill_holes
from scipy.ndimage.interpolation import map_coordinates
import camera_info_manager as cinfo_manager
import matplotlib.pyplot as plt
import open3d
from scipy.interpolate import griddata
import argparse
from dataset_utils import use_data_dirs, handoff_data_dirs
import logging
osp = os.path

class RegisterRGB(object):
  def __init__(self, data_dir, use_depth=False):
    self.data_dir = osp.join(osp.expanduser(data_dir))
    self.use_depth = use_depth

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
    logger = logging.getLogger(__name__)
    base_dir = osp.join(self.data_dir, session_name, object_name)
    if not osp.isfile(osp.join(base_dir, 'object_name.txt')):
      logger.warn('{:s} does not contain object_name.txt, skipping'.format(base_dir))
      return
    rgb_dir = osp.join(base_dir, 'rgb_images')
    depth_dir = osp.join(base_dir, 'depth_images')

    # thermal camera info
    thermal_dir = osp.join(base_dir, 'thermal_images')
    thermal_im_size, thermal_K = self._load_thermal_cinfo(thermal_dir)

    _, _, rgb_filenames = os.walk(rgb_dir).next()
    rgb_filenames = [fn for fn in rgb_filenames if 'registered' not in fn]
    depth_filenames = rgb_filenames[:]
    done = True
    for rgb_filename, depth_filename in \
      zip(sorted(rgb_filenames), sorted(depth_filenames)):
      u_thermal, v_thermal = np.meshgrid(np.arange(thermal_im_size[0]),
          np.arange(thermal_im_size[1]))
      uv_thermal = np.vstack((
        u_thermal.ravel(), v_thermal.ravel(), np.ones(u_thermal.size))).astype(int)
      xyz_thermal = np.dot(np.linalg.inv(thermal_K), uv_thermal)
      if self.use_depth:  # use full depth
        depth_filename = osp.join(depth_dir, depth_filename)
        im_depth = cv2.imread(depth_filename, -1)
        im_depth = fill_holes(im_depth)
        d_thermal = im_depth.ravel() / 1000.0
        xyz_thermal *= d_thermal[np.newaxis, :]
        xyz_rgb = np.dot(self.T_rgb_t,
          np.vstack((xyz_thermal, np.ones(xyz_thermal.shape[1]))))
      else:  # use only normalized camera coordinates
        xyz_rgb = xyz_thermal

      uv_rgb = np.dot(self.rgb_K, xyz_rgb[:3])
      uv_rgb = uv_rgb[:2] / uv_rgb[2]

      # interpolate
      rgb_filename = osp.join(rgb_dir, rgb_filename)
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

      rgb_filename, ext = rgb_filename.split('.')
      rgb_filename = '{:s}_registered.{:s}'.format(rgb_filename, ext)
      im_thermal *= 255
      done = done and cv2.imwrite(rgb_filename, im_thermal[:, :, ::-1].astype(np.uint8))
      if not done:
        logger.error('### WARN: Could not write {:s}'.format(rgb_filename))
      else:
        logger.info('Written {:s}'.format(rgb_filename))

    return done

  def create_object_masks(self, object_name, session_name, dilate_size=15,
      show=False):
    logger = logging.getLogger(__name__)
    # DEPRECATED: use generate_object_masks.py instead
    # TODO: Bug: no mask for excluded views, because their segmented object
    # is not saved
    base_dir = osp.join(self.data_dir, session_name, object_name)
    pc_dir = osp.join(base_dir, 'pointclouds')

    # thermal camera info
    thermal_dir = osp.join(base_dir, 'thermal_images')
    thermal_im_size, thermal_K = self._load_thermal_cinfo(thermal_dir)

    _, _, pc_filenames = os.walk(pc_dir).next()
    done = True
    for pc_filename in pc_filenames:
      if 'segmented_object' not in pc_filename:
        continue
      view_id = pc_filename.split('_')[0]

      pc_filename = osp.join(pc_dir, pc_filename)
      xyz_kinect = open3d.read_point_cloud(pc_filename)
      xyz_kinect = np.asarray(xyz_kinect.points).T

      xyz_thermal = np.dot(np.linalg.inv(self.T_rgb_t),
        np.vstack((xyz_kinect, np.ones(xyz_kinect.shape[1]))))

      uv_thermal = np.dot(thermal_K, xyz_thermal[:3])
      uv_thermal = uv_thermal[:2] / uv_thermal[2]
      uv_thermal = np.round(uv_thermal).astype(np.int)
      idx = np.logical_and(uv_thermal >= 0,
        uv_thermal < np.asarray([thermal_im_size[0], thermal_im_size[1]])[:, np.newaxis])
      idx = np.logical_and(*idx)
      uv_thermal = uv_thermal[:, idx]

      mask = np.zeros((thermal_im_size[1], thermal_im_size[0]))
      mask[uv_thermal[1], uv_thermal[0]] = 1

      if show:
        plt.figure()
        plt.imshow(mask)
        plt.show()

      mask_filename = '{:s}_mask.png'.format(view_id)
      mask_filename = osp.join(thermal_dir, mask_filename)
      mask = (mask * 255).astype(np.uint8)
      kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
        (dilate_size, dilate_size))
      mask = cv2.dilate(mask, kernel)
      done = done and cv2.imwrite(mask_filename, mask)
      if not done:
        logger.error('### WARN: Could not write {:s}'.format(mask_filename))

    return done


def register_images(object_names, session_nums, instruction):
  logger = logging.getLogger(__name__)
  data_dirs = use_data_dirs if instruction == 'use' else handoff_data_dirs
  for session_num in session_nums:
    session_name = 'full{:d}_{:s}'.format(session_num, instruction)
    data_dir = data_dirs[session_num-1]
    rr = RegisterRGB(use_depth=True, data_dir=data_dir)
    if object_names is None:
      ons = next(os.walk(osp.join(data_dir, session_name)))[1]
    else:
      ons = object_names[:]
    for object_name in ons:
      try:
        rr.register_images(object_name=object_name, session_name=session_name)
        logger.info('Processed {:s} - {:s}'.format(session_name, object_name))
      except:
        logger.error('Could not process {:s} - {:s}'.format(session_name,
          object_name))


if __name__ == '__main__':
  logging.basicConfig(level=logging.WARN)
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_names', default=None, help='comma separated')
  parser.add_argument('--session_nums', required=True, help='comma separated')
  parser.add_argument('--instruction', required=True)
  args = parser.parse_args()

  session_nums = [int(n) for n in args.session_nums.split(',')]
  object_names = args.object_names
  if object_names is not None:
    object_names = object_names.split(',')
  register_images(object_names, session_nums, args.instruction)

  # rr.create_object_masks(object_name='binoculars', session_name='full2_use',
  #   show=True)

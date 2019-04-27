"""
Generates the object mask by projecting the object model into each image
"""

import numpy as np
import open3d
import os
import camera_info_manager as cinfo_manager
import argparse
from copy import copy
import open3d
import cv2
from render_depth_maps import render_depth_maps
import init_paths
from depth_proc import fill_holes
import matplotlib.pyplot as plt
import logging
from IPython.core.debugger import set_trace
osp = os.path

default_models_dir = osp.join('..', 'data', 'contactdb_3d_models')
default_dilation = 15


def generate_all_object_masks(object_names, session_name, base_dir,
    models_dir, dilate_size):
  logger = logging.getLogger(__name__)
  if object_names is None:
    object_names = next(os.walk(osp.join(base_dir, session_name)))[1]
  for object_name in object_names:
    try:
      generate_object_masks(object_name, session_name, base_dir, models_dir,
          dilate_size)
      logger.info('Processed {:s} - {:s}'.format(session_name, object_name))
    except:
      logger.error('Could not process {:s} - {:s}'.format(session_name, object_name))
      

def generate_object_masks(object_name, session_name, base_dir,
    models_dir=default_models_dir, dilate_size=default_dilation):
  logger = logging.getLogger(__name__)
  data_dir = osp.join(base_dir, session_name, object_name)
  try:
    with open(osp.join(data_dir, 'object_name.txt'), 'r') as f:
      object_name = f.readline().rstrip()
  except IOError:
    logger.warn('{:s} does not have object_name.txt, skipping'.format(data_dir))
    return
  mesh_filename = osp.join(models_dir, '{:s}.ply'.format(object_name))

  # names of views
  pose_dir = osp.join(data_dir, 'poses')
  names = []
  for filename in os.listdir(pose_dir):
    if '.txt' not in filename:
      continue
    if 'camera_pose' not in filename:
      continue
    name = '_'.join(filename.split('.')[0].split('_')[2:])
    names.append(name)
  names = sorted(names)

  # camera extrinsics
  Ts = []
  for name in names:
    filename = osp.join(pose_dir, 'camera_pose_{:s}.txt'.format(name))
    T = np.eye(4)
    with open(filename, 'r') as f:
      f.readline()
      T[0, 3] = float(f.readline().strip())
      T[1, 3] = float(f.readline().strip())
      T[2, 3] = float(f.readline().strip())
      f.readline()
      f.readline()
      T[0, :3] = [float(v) for v in f.readline().strip().split()]
      T[1, :3] = [float(v) for v in f.readline().strip().split()]
      T[2, :3] = [float(v) for v in f.readline().strip().split()]
    Ts.append(np.linalg.inv(T))

  rgb_im_dir = osp.join(data_dir, 'thermal_images')
  im = cv2.imread(osp.join(rgb_im_dir, '{:s}.png'.format(names[0])))
  im_shape = im.shape

  # camera intrinsic
  cinfo_filename = cinfo_manager.getPackageFileName(
    'package://contactdb_utils/calibrations/boson.yaml')
  cinfo = cinfo_manager.loadCalibrationFile(cinfo_filename, 'boson')
  h_scaling = float(im_shape[0]) / cinfo.height
  w_scaling = float(im_shape[1]) / cinfo.width
  K = np.asarray(cinfo.K)
  K[0] *= w_scaling
  K[2] *= w_scaling
  K[4] *= h_scaling
  K[5] *= h_scaling
  K = K.reshape((3,3))
  intrinsic = open3d.PinholeCameraIntrinsic(im_shape[1], im_shape[0], K[0,0],
    K[1,1], K[0,2], K[1,2])

  # render depth maps 
  depth_ims = render_depth_maps(mesh_filename, intrinsic, Ts)

  # save masks
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
      (dilate_size, dilate_size))
  for name, depth_im in zip(names, depth_ims):
    mask_filename = osp.join(data_dir, 'thermal_images',
      '{:s}_mask.png'.format(name))
    mask = np.uint8(255 * (depth_im > 0))
    mask = cv2.dilate(mask, kernel)
    if cv2.imwrite(mask_filename, mask.astype(np.uint8)):
      logger.info('Written {:s}'.format(mask_filename))


if __name__ == '__main__':
  logging.basicConfig(level=logging.WARN)
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', default=None,
      help="comma separated list of objects")
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--data_dir',
    default=osp.join('..', 'data', 'contactdb_data'))
  parser.add_argument('--models_dir',
    default=default_models_dir)
  parser.add_argument('--dilate_size', type=int, default=default_dilation)
  args = parser.parse_args()

  object_names = args.object_name
  if object_names is not None:
    object_names = object_names.split(',')
  generate_all_object_masks(object_names, args.session_name,
    osp.expanduser(args.data_dir), osp.expanduser(args.models_dir),
    args.dilate_size)

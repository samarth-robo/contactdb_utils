"""
Uses open3D colormap optimization to do texture mapping
"""

import numpy as np
import open3d
import os
import camera_info_manager as cinfo_manager
import argparse
from copy import copy
import open3d
from render_depth_maps import render_depth_maps
import init_paths
from depth_proc import fill_holes
from show_textured_mesh import show_object_mesh
import dataset_utils
from IPython.core.debugger import set_trace
osp = os.path


def map_texture(object_name, session_name, base_dir, models_dir,
    debug_mode=False, show_textured_mesh=False,
    depth_thresh_for_visibility=1e-2, depth_thresh_for_discontinuity=0.035,
    max_vertex_normal_angle=70, real_depth_maps=False):
  data_dir = osp.join(base_dir, session_name, object_name)
  try:
    with open(osp.join(data_dir, 'object_name.txt'), 'r') as f:
      object_name = f.readline().rstrip()
  except IOError:
    print('{:s} does not have object_name.txt, skipping'.format(data_dir))
    return

  # read mesh file
  mesh_filename = osp.join(models_dir, '{:s}.ply'.format(object_name))
  mesh = open3d.read_triangle_mesh(mesh_filename)
  if not mesh.has_vertex_normals():
    mesh.compute_vertex_normals()

  # names of views
  rgb_im_dir = osp.join(data_dir, 'thermal_images')
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

  # RGB images
  rgb_ims = []
  im_intensities = []
  for name in names:
    rgb_im = open3d.read_image(osp.join(rgb_im_dir, '{:s}.png'.format(name)))
    rgb_im = np.asarray(rgb_im)
    rgb_ims.append(rgb_im)
    im_shape = rgb_im.shape
    intensity = rgb_im[:200, :200, 0].mean()
    im_intensities.append(intensity)
  target_intensity = np.mean(im_intensities)
  for i, rgb_im in enumerate(rgb_ims):
    rgb_im = rgb_im * target_intensity / im_intensities[i]
    rgb_im = np.clip(rgb_im, a_min=0, a_max=255)
    rgb_ims[i] = open3d.Image(rgb_im.astype(np.uint8))

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

  if real_depth_maps:  # use registered Kinect depthmaps
    depth_im_dir = osp.join(data_dir, 'depth_images')
    depth_ims = []
    for name in names:
      depth_im_filename = osp.join(depth_im_dir, '{:s}.png'.format(name))
      depth_im = open3d.read_image(depth_im_filename)
      depth_im = np.uint16(fill_holes(depth_im))
      depth_ims.append(depth_im)
  else:  # create depth maps by rendering
    depth_ims = render_depth_maps(mesh_filename, intrinsic, Ts)

  # create RGB-D images
  rgbds = []
  for depth_im, rgb_im, T in zip(depth_ims, rgb_ims, Ts):
    depth_im = open3d.Image(depth_im)
    rgbds.append(open3d.create_rgbd_image_from_color_and_depth(rgb_im,
      depth_im, convert_rgb_to_intensity=False))
    if debug_mode:
      pc = open3d.create_point_cloud_from_rgbd_image(rgbds[-1], intrinsic)
      tmesh = copy(mesh)
      tmesh.transform(T)
      geoms = [pc]
      if show_textured_mesh:
        geoms.append(tmesh)
      open3d.draw_geometries(geoms)

  # create trajectory for texture mapping
  traj = open3d.PinholeCameraTrajectory()
  traj.extrinsic = open3d.Matrix4dVector(np.asarray(Ts))
  traj.intrinsic = intrinsic

  # do texture mapping!
  option = open3d.ColorMapOptmizationOption()
  option.maximum_iteration = 300
  option.depth_threshold_for_visiblity_check = depth_thresh_for_visibility
  option.depth_threshold_for_discontinuity_check = \
      depth_thresh_for_discontinuity
  option.half_dilation_kernel_size_for_discontinuity_map = 0
  option.max_angle_vertex_normal_camera_ray = max_vertex_normal_angle
  open3d.color_map_optimization(mesh, rgbds, traj, option)

  if not debug_mode:  # write result as a PLY file
    mesh_filename = osp.join(data_dir, 'thermal_images',
      '{:s}_textured.ply'.format(object_name))
    open3d.write_triangle_mesh(mesh_filename, mesh)
    print('Written {:s}'.format(mesh_filename))
    if show_textured_mesh:
      show_object_mesh(data_dir)


def map_all_textures(object_names, session_nums, instructions,
    models_dir, debug_mode, show_textured_mesh,
    depth_thresh_for_visibility, depth_thresh_for_discontinuity,
    max_vertex_normal_angle, real_depth_maps=False):
  for instruction in instructions:
    data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction))
    for session_num in session_nums:
      data_dir = data_dirs[int(session_num)-1]
      for object_name in object_names:
        session_name = 'full{:s}_{:s}'.format(session_num, instruction)
        map_texture(object_name, session_name,
            data_dir, models_dir,
            show_textured_mesh=show_textured_mesh, debug_mode=debug_mode,
            depth_thresh_for_visibility=depth_thresh_for_visibility,
            depth_thresh_for_discontinuity=depth_thresh_for_discontinuity,
            max_vertex_normal_angle=max_vertex_normal_angle)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_names', required=True, help='comma separated')
  parser.add_argument('--session_nums', default=None, help='comma or - separated')
  parser.add_argument('--instructions', default=None, help='comma separated')
  parser.add_argument('--models_dir',
    default=osp.join('..', 'data', 'contactdb_3d_models'))
  parser.add_argument('--visibility_thresh', type=float, default=1e-2)
  parser.add_argument('--discontinuity_thresh', type=float, default=0.035)
  parser.add_argument('--vertex_normal_angle', type=float, default=70)
  args = parser.parse_args()

  object_names = args.object_names.split(',')
  session_nums = args.session_nums
  if session_nums is not None:
    if '-' in session_nums:
      start, end = session_nums.split('-')
      session_nums = ['{:d}'.format(s) for s in range(int(start), int(end)+1)]
    else:
      session_nums = session_nums.split(',')
  else:
    session_nums = ['{:d}'.format(s) for s in range(1, 51)]
  instructions = args.instructions
  if instructions is not None:
    instructions = instructions.split(',')
  else:
    instructions = ['use', 'handoff']

  show = (len(object_names) == 1) and (len(session_nums) == 1) and \
      (len(instructions) == 1)

  # debug verbosity
  # open3d.set_verbosity_level(open3d.VerbosityLevel.Debug)
  map_all_textures(object_names, session_nums, instructions,
      osp.expanduser(args.models_dir), show_textured_mesh=show,
      debug_mode=False,
      depth_thresh_for_visibility=args.visibility_thresh,
      depth_thresh_for_discontinuity=args.discontinuity_thresh,
      max_vertex_normal_angle=args.vertex_normal_angle)

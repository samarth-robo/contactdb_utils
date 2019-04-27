"""
This script processes the data in an entire session, e.g. full19_use.
Before running this script:
1. Run make_object_dirs.py for the session
2. Go through all the object folders in the session and record the IDs (e.g. 04)
of the views unsuitable for ICP in that folder's excluded_views.txt
3. Objects which have been captured from multiple angles have multiple folders
e.g. banana and banana-1. For view folder, delete its merge.txt file if you
don't want this view to be merged with other views
"""
import sys
import os
import argparse
import random
import numpy as np
import subprocess
import init_paths
from texture_mapping import map_texture
from generate_camera_poses import PoseGenerator
from generate_object_masks import generate_object_masks
from reset_poses import reset_poses
from copy_cams import copy_cams
import dataset_utils
from Logger import Logger
from IPython.core.debugger import set_trace
osp = os.path

symmetric_objects = ['apple', 'bowl', 'cup', 'flashlight', 'water_bottle',
  'wine_glass', 'cylinder_small', 'cylinder_medium', 'cylinder_large',
  'sphere_small', 'sphere_medium', 'sphere_large',
  'torus_small', 'torus_medium', 'torus_large']
restrict_rotation = ['apple', 'flute', 'light_bulb', 'toothbrush',
  'cell_phone', 'stapler', 'door_knob', 'toothpaste',
  'sphere_small', 'sphere_medium', 'sphere_large']
only_xy = ['bowl', 'torus_small']
azim_search_range = {so: 0 for so in symmetric_objects}
# Objects for which you want to prevent 180-degree switching
switch_objects = ['cell_phone', 'door_knob', 'flute', 'knife', 'stapler',
  'toothbrush', 'utah_teapot', 'cube_small', 'cube_large', 'cube_medium',
  'pyramid_small', 'pyramid_medium', 'pyramid_large', 'alarm_clock',
  'wristwatch', 'palm_print']
small_azim_search_range = 0
for so in switch_objects:
  azim_search_range[so] = small_azim_search_range
# Large objects which need plane estimate from other objects
large_objects = ['pan', 'cube_large', 'pyramid_large', 'piggy_bank']
# objects with large planar surfaces that can be distorted by Kinect v2
distorted_objects = [
    'pyramid_small', 'pyramid_medium', 'pyramid_large',
    'cube_small', 'cube_medium', 'cube_large'
    ]
# objects with flat surfaces
flat_objects = ['knife', 'toothpaste', 'cube_large', 'bowl', 'pyramid_large']
flat_object_vertex_normal_angle = 90
flat_object_disc_thresh = 0.07
ignore_objects = ['usb_drive', 'wrench']
special_rotations = {'light_bulb': [4, -14], 'toothpaste': [4, -4]}

default_models_dir = osp.expanduser(osp.join('..', 'data', 'contactdb_3d_models'))
default_icp_exec = osp.expanduser(osp.join('..', 'ICP_build', 'grasp_processor'))

def process_session(session_name, data_dir, models_dir=default_models_dir,
    icp_executable=default_icp_exec, include_objects=None):
  # include_objects is useful for re-processing some particular objects
  # set to None to process all objects
  # include_objects = ['palm_print']

  pose_generator = PoseGenerator(data_dir)

  object_names = []
  session_dir = osp.join(data_dir,session_name)
  for object_name in sorted(os.listdir(session_dir)):
    object_dir = osp.join(session_dir, object_name)
    if not osp.isdir(object_dir):
      continue

    object_name_filename = osp.join(object_dir, 'object_name.txt')
    try:
      with open(object_name_filename, 'r') as f:
        real_object_name = f.readline().rstrip()
    except IOError:
      print('{:s} does not have object_name.txt, skipping'.format(object_dir))
      continue

    if real_object_name in ignore_objects:
      print('Ignoring {:s}'.format(object_name))
      continue

    if include_objects is not None:
      if object_name not in include_objects:
        continue

    object_names.append(real_object_name)
    
    # Clean out old poses
    print('Processing {:s}'.format(object_name))
    reset_poses(object_dir)

    if real_object_name in special_rotations:
      flip_filename = osp.join(object_dir, 'object_flip.txt')
      flip = np.loadtxt(flip_filename)
      idx, angle = special_rotations[real_object_name]
      flip[idx] = angle
      np.savetxt(flip_filename, flip[np.newaxis, :], fmt='%d')

    # Run automated ICP
    cmd = [icp_executable, data_dir, object_name, session_name]
    symmetric = real_object_name in symmetric_objects
    if symmetric:
      cmd.append('--symmetric')
    no_rollpitch = real_object_name in restrict_rotation
    if no_rollpitch:
      cmd.append('--no_rollpitch')
    if real_object_name in only_xy:
      cmd.append('--only_xy')
    if real_object_name in distorted_objects:
      cmd.append('--distorted')
    first_view_filename = osp.join(object_dir, 'first_view.txt')
    anchor_view = None
    with open(first_view_filename, 'r') as f:
      lines = [line.strip() for line in f]
    if len(lines) > 0:
      anchor_view = lines[0]
    azim_range = -1
    if anchor_view is not None:
      azim_range = small_azim_search_range
    if real_object_name in azim_search_range:
      azim_range = azim_search_range[real_object_name]
    if azim_range >= 0:
      cmd.append('--azim_range {:d}'.format(azim_range))
    if real_object_name in large_objects:
      try:
        cmd.append('--plane_from {:s}'.format(random.choice(object_names[:-1])))
      except:
        cmd.append('--plane_from toothbrush')
    cmd = ' '.join(cmd)
    try:
      p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
      out = p.communicate()[0]
    except Exception as e:
      print('#### ERROR IN POSE ESTIMATION FOR {:s}: {:s} @@@@'.
          format(object_dir, e))
    print(out)

    # Interpolate poses
    try:
      pose_generator.estimate_poses(object_name, session_name, ignore_gt=False,
        symmetric=symmetric, anchor_view=anchor_view)
    except Exception as e:
      print('#### ERROR IN POSE PROCESSING FOR {:s}: {:s} @@@@'.
          format(object_dir, e))

  # merge views
  object_names = list(set(object_names))
  for object_name in object_names:
    copy_cams(object_name, session_name, data_dir)

  # texture map
  for object_name in sorted(os.listdir(session_dir)):
    object_dir = osp.join(session_dir, object_name)
    real_object_name = object_name.split('-')[0]
    if real_object_name not in object_names:
      continue
    kwargs = {}
    if real_object_name in flat_objects:
      kwargs['max_vertex_normal_angle'] = flat_object_vertex_normal_angle
      kwards['depth_thresh_for_discontinuity'] = flat_object_disc_thresh
    if not osp.isdir(object_dir):
      continue
    try:
      map_texture(object_name, session_name, data_dir, models_dir, **kwargs)
    except Exception as e:
      print('#### ERROR IN TEXTURE MAPPING FOR {:s}: {:s} @@@@'.
          format(object_dir, e))

  # generate masks
  for object_name in sorted(os.listdir(session_dir)):
    object_dir = osp.join(session_dir, object_name)
    real_object_name = object_name.split('-')[0]
    if real_object_name not in object_names:
      continue
    try:
      generate_object_masks(object_name, session_name, data_dir)
    except Exception as e: 
      print('#### ERROR IN MASK GENERATION FOR {:s} @@@@'.
          format(object_dir, e))
      

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--models_dir', help='Models Directory',
    default=default_models_dir)
  parser.add_argument('--icp_executable', help='Full path to ICP executable',
    default=default_icp_exec)
  parser.add_argument('--object_name',
      help='Only process specific objects (comma-separate their names)',
      default=None)
  args = parser.parse_args()

  session_num, instruction = args.session_name.split('_')
  session_num = session_num.replace('full', '')
  data_dir = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction)) 
  data_dir = data_dir[int(session_num)-1]

  log_filename = osp.join(data_dir, args.session_name,
      'processing_log.txt')
  log_filename = osp.expanduser(log_filename)
  print('Logging to {:s}'.format(log_filename))
  stdout = Logger(log_filename)
  sys.stdout = stdout

  include_objects = args.object_name
  if include_objects is not None:
    include_objects = include_objects.split(',')
  process_session(session_name=args.session_name,
    data_dir=data_dir,
    models_dir=osp.expanduser(args.models_dir),
    icp_executable=osp.expanduser(args.icp_executable),
    include_objects=include_objects)

  stdout.delink()

import numpy as np
import os
import logging
import argparse
import dataset_utils
from IPython.core.debugger import set_trace
osp = os.path


def decide_src_dst(merge_dirs):
  if len(merge_dirs) == 1:
    return [], merge_dirs
  src_dirs = []
  dst_dirs = []
  for d in merge_dirs:
    poses_dir = osp.join(d, 'poses')
    for f in os.walk(poses_dir).next()[-1]:
      if 'view' in f:
        dst_dirs.append(d)
        break
    else:
      src_dirs.append(d)
  return src_dirs, dst_dirs


def detect_file_discrepancies(session_name, data_dir):
  logger = logging.getLogger(__name__)
  base_dir = osp.join(data_dir, session_name)
  for object_dir in next(os.walk(base_dir))[1]:
    object_dir = osp.join(base_dir, object_dir)
    object_name_filename = osp.join(object_dir, 'object_name.txt')
    try:
      with open(object_name_filename, 'r') as f:
        object_name = f.readline().strip()
    except IOError:
      logger.warn('Skipping {:s}'.format(object_dir))
      continue

    pose_dir = osp.join(object_dir, 'poses')
    pose_count = 0
    for fname in next(os.walk(pose_dir))[-1]:
      if 'camera_pose' not in fname:
        continue
      pose_count += 1
    thermal_dir = osp.join(object_dir, 'thermal_images')
    thermal_count = 0
    for fname in next(os.walk(thermal_dir))[-1]:
      if ('.png' not in fname) or ('mask' in fname):
        continue
      thermal_count += 1
    if pose_count != thermal_count:
      logger.warning('{:s} has unequal number of poses and thermal images'.
          format(object_dir))


def detect_merge_files(session_name, data_dir):
  logger = logging.getLogger(__name__)
  base_dir = osp.join(data_dir, session_name)

  object_dirs = {}
  for object_dir in next(os.walk(base_dir))[1]:
    object_dir = osp.join(base_dir, object_dir)
    object_name_filename = osp.join(object_dir, 'object_name.txt')
    try:
      with open(object_name_filename, 'r') as f:
        object_name = f.readline().strip()
    except IOError:
      logger.warn('Skipping {:s}'.format(object_dir))
      continue
    if object_name not in object_dirs:
      object_dirs[object_name] = [object_dir]
    else:
      object_dirs[object_name].append(object_dir)

  for oname, odirs in object_dirs.items():
    if len(odirs) > 1:
      nonmerge_dirs = [od for od in odirs if not
          osp.isfile(osp.join(od, 'merge.txt'))]
      for d in nonmerge_dirs:
        logger.info('{:s} does not have merge.txt'.format(d))

      merge_dirs = [od for od in odirs if
          osp.isfile(osp.join(od, 'merge.txt'))]
      src_dirs, dst_dirs = decide_src_dst(merge_dirs)
      if len(dst_dirs) > 1:
        msg = ', '.join(dst_dirs)
        logger.error('{:s} are all destination dirs!'.format(msg))
      elif len(dst_dirs) == 1:
        with open(osp.join(dst_dirs[0], 'merge.txt'), 'w') as f:
          f.write('1\n')
        logger.info('{:s} is destination dir for {:s}'.
            format(dst_dirs[0], oname))
      else:
        logger.error('No destination dirs for {:s} {:s}!'.
            format(oname, session_name))


def detect_defects(session_nums, instruction):
  logger = logging.getLogger(__name__)
  data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction))
  for session_id in session_nums:
    session_name = 'full{:s}_{:s}'.format(session_id, instruction)
    logger.info('Session: {:s}'.format(session_name))
    idx = int(session_id) - 1
    data_dir = data_dirs[idx]
    detect_file_discrepancies(session_name, data_dir)
    detect_merge_files(session_name, data_dir)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--session_nums', required=True, help='comma separated')
  parser.add_argument('--instruction', required=True)
  args = parser.parse_args()

  session_nums = args.session_nums
  if session_nums is not None:
    session_nums = session_nums.split(',')

  logging.basicConfig(level=logging.INFO)
  detect_defects(session_nums, args.instruction)

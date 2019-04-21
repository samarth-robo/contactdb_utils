"""
script to copy the thermal images and cam files from all directories containing
rotated view to the main directory
"""
import os
import shutil
import argparse
osp = os.path


def copy_cams(object_name, session_name, base_dir):
  base_dir = osp.expanduser(base_dir)
  base_dir = osp.join(base_dir, session_name)

  # find merges
  merge_dirs = []
  for dir_name in os.listdir(base_dir):
    if object_name not in dir_name:
      continue
    dir_name = osp.join(base_dir, dir_name)
    if not osp.isdir(dir_name):
      continue
    merge_filename = osp.join(dir_name, 'merge.txt')
    if not osp.isfile(merge_filename):
      continue
    merge_dirs.append(dir_name)

  if len(merge_dirs) < 2:
    return
  merge_dirs = sorted(merge_dirs)
  print('Merging all {:s} to {:s}'.format(object_name, merge_dirs[0]))

  # execute merges
  for dir_name in merge_dirs[1:]:
    if '-' in dir_name:
      _, count = dir_name.split('-')
    else:
      count = 0

    print('Copying {:s}'.format(dir_name))

    # copy thermal images and cam files
    src_dir_name = osp.join(dir_name, 'thermal_images')
    dst_dir_name = osp.join(merge_dirs[0], 'thermal_images')
    for src_filename in os.listdir(src_dir_name):
      if ('.cam' in src_filename) or ('.png' in src_filename):
        shutil.copyfile(osp.join(src_dir_name, src_filename),
          osp.join(dst_dir_name, 'view_{:s}_{:s}'.format(count, src_filename)))

    # copy depth images
    src_dir_name = osp.join(dir_name, 'depth_images')
    dst_dir_name = osp.join(merge_dirs[0], 'depth_images')
    for src_filename in os.listdir(src_dir_name):
      if '.png' in src_filename:
        shutil.copyfile(osp.join(src_dir_name, src_filename),
          osp.join(dst_dir_name, 'view_{:s}_{:s}'.format(count, src_filename)))

    # copy registered RGB images
    src_dir_name = osp.join(dir_name, 'rgb_images')
    dst_dir_name = osp.join(merge_dirs[0], 'rgb_images')
    for src_filename in os.listdir(src_dir_name):
      if 'registered.png' in src_filename:
        shutil.copyfile(osp.join(src_dir_name, src_filename),
          osp.join(dst_dir_name, 'view_{:s}_{:s}'.format(count, src_filename)))

    # copy poses
    src_dir_name = osp.join(dir_name, 'poses')
    dst_dir_name = osp.join(merge_dirs[0], 'poses')
    for src_filename in os.listdir(src_dir_name):
      if 'camera_pose' not in src_filename:
        continue
      _, _, idx = src_filename.split('.')[0].split('_')
      shutil.copyfile(osp.join(src_dir_name, src_filename),
        osp.join(dst_dir_name, 'camera_pose_view_{:s}_{:s}.txt'.format(count, idx)))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--data_dir',
    default=osp.join('~', 'deepgrasp_data', 'data'))
  args = parser.parse_args()

  copy_cams(args.object_name, args.session_name, args.data_dir)

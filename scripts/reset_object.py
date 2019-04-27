"""
Resets all computed information for an object, but retains the data extracted
from the bag
"""
import os
import argparse
from reset_poses import reset_poses
osp = os.path


def reset_object(object_name, session_name, data_dir):
  object_dir = osp.join(data_dir, session_name, object_name)
  # reset the poses
  reset_poses(object_dir)

  # delete cam files, masks, textured mesh, and other views' texture files
  thermal_dir = osp.join(object_dir, 'thermal_images')
  for filename in os.listdir(thermal_dir):
    filename = osp.join(thermal_dir, filename)
    if not osp.isfile(filename):
      continue

    if not (('.cam' in filename) or ('view_' in filename) or
            ('_mask' in filename) or ('_textured.ply' in filename)):
      continue
    os.remove(filename)

  # delete other views' depth maps
  depth_dir = osp.join(object_dir, 'depth_images')
  for filename in os.listdir(depth_dir):
    if not 'view_' in filename:
      continue
    filename = osp.join(depth_dir, filename)
    if not osp.isfile(filename):
      continue
    os.remove(filename)

  # delete registered RGB images
  rgb_dir = osp.join(object_dir, 'rgb_images')
  for filename in os.listdir(rgb_dir):
    filename = osp.join(rgb_dir, filename)
    if not osp.isfile(filename):
      continue
    if not '_registered' in filename:
      continue
    os.remove(filename)

  # delete segmented object point clouds
  pc_dir = osp.join(object_dir, 'pointclouds')
  for filename in os.listdir(pc_dir):
    filename = osp.join(pc_dir, filename)
    if not osp.isfile(filename):
      continue
    if not '_segmented_object.pcd' in filename:
      continue
    os.remove(filename)

  print('Reset object {:s}'.format(object_dir))


def reset_all_objects(session_name, data_dir):
  for object_name in os.listdir(osp.join(data_dir, session_name)):
    if object_name == 'bags':
      continue
    reset_object(object_name, session_name, data_dir)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name')
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--data_dir',
      default=osp.join('..', 'data', 'contactdb_data'))
  args = parser.parse_args()

  data_dir = osp.expanduser(args.data_dir)
  if args.object_name is not None:
    reset_object(args.object_name, args.session_name, data_dir)
  else:
    reset_all_objects(args.session_name, data_dir)

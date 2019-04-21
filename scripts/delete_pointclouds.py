"""
This script deletes the pointclouds from a directory
Run it after all processing is done for that directory, since the pointclouds
are no longer needed, and occupy a lot of disk space
"""
import argparse
import os
import dataset_utils
osp = os.path

def delete_object_pointclouds(base_dir):
  base_dir = osp.join(base_dir, 'pointclouds')
  if not osp.isdir(base_dir):
    print('{:s} is not a directory!'.format(base_dir))
    return
  for filename in os.listdir(base_dir):
    filename = osp.join(base_dir, filename)

    if '.pcd' not in filename:
      continue
    os.remove(filename)
  print('Deleted pointclouds for {:s}'.format(base_dir))


def delete_all_pointclouds(object_names, session_nums, instruction):
  data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction))
  for session_num in session_nums:
    data_dir = data_dirs[int(session_num)-1]
    # data_dir = '/home/samarth/Dropbox (GaTech)/contactdb_data/'
    session_name = 'full{:s}_{:s}'.format(session_num, instruction)
    base_dir = osp.join(data_dir, session_name)
    for object_name in os.listdir(base_dir):
      if object_name == 'bags':
        continue
      if object_names is not None:
        real_object_name = object_name.split('-')[0]
        if real_object_name not in object_names:
          continue
      delete_object_pointclouds(osp.join(base_dir, object_name))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_names', default=None, help='comma separated')
  parser.add_argument('--session_nums', required=True, help='comma or - separated')
  parser.add_argument('--instruction', required=True)
  args = parser.parse_args()

  session_nums = args.session_nums
  if '-' in session_nums:
    start, end = session_nums.split('-')
    session_nums = ['{:d}'.format(s) for s in range(int(start), int(end)+1)]
  else:
    session_nums = args.session_nums.split(',')
  object_names = args.object_names
  if object_names is not None:
    object_names = object_names.split(',')

  delete_all_pointclouds(object_names, session_nums, args.instruction)

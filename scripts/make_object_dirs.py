"""
Creates directories for objects in a particular participant session, in which
the play_bag launch file can be run
"""
import os
import argparse
from subprocess import Popen
import dataset_utils
from IPython.core.debugger import set_trace
osp = os.path

ignored_objects = ['usb_drive', 'wrench']

def make_dir(base_dir, object_name, bag_filename, count=0,
    re_extract=False):
  session_name = base_dir.rstrip('/').split('/')[-1]
  if count > 0:
    object_name += '-{:d}'.format(count)
  dir_name = osp.join(base_dir, object_name)

  if re_extract and not osp.isdir(dir_name):
    print('re_extract = True but {:s} does not exist!'.format(dir_name))
  elif not re_extract and osp.isdir(dir_name):
    print('re_extract = False but {:s} already exists!'.format(dir_name))
  else:
    if not osp.isdir(dir_name):
      os.makedirs(dir_name)
      recording_filename = osp.join(dir_name, 'recording.txt')
      s = osp.join('..', '..', 'rosbags', session_name, bag_filename)
      with open(recording_filename, 'w') as f:
        f.write('{:s}\n'.format(s))

      merge_filename = osp.join(dir_name, 'merge.txt')
      with open(merge_filename, 'w') as f:
        pass

    # play the bag file
    data_dir = osp.join(base_dir, '..')
    # determine if the session was recorded with the old system
    old = int(session_name.split('_')[0].strip('full')) <= 4
    cmd = 'roslaunch deepgrasp_utils play_bag.launch data_dir:={:s} ' \
          'object_name:={:s} p_id:={:s}'.format(data_dir, object_name,
      session_name)
    if old:
      cmd += ' old:=true'
    if re_extract:
      cmd += ' re_extract:=true'
    Popen(cmd, shell=True).wait()


def create_dirs(instructions, session_nums, object_names, re_extract):
  if instructions is None:
    instructions = ['handoff', 'use']
  else:
    instructions = instructions.split(',')
  if session_nums is None:
    n_sessions = len(dataset_utils.use_data_dirs)
    session_nums = ['{:d}'.format(s) for s in range(1, n_sessions+1)]
  elif '-' in session_nums:
    start, end = session_nums.split('-')
    session_nums = ['{:d}'.format(s) for s in range(int(start), int(end)+1)]
  else:
    session_nums = session_nums.split(',')

  if object_names is not None:
    object_names = object_names.split(',')

  for instruction in instructions:
    data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction))
    for session_num in session_nums:
      session_name = 'full{:s}_{:s}'.format(session_num, instruction)
      data_dir = data_dirs[int(session_num)-1]
      base_dir = osp.join(data_dir, session_name)
      bags_dir = osp.join(data_dir, 'rosbags', session_name)

      bags = {}
      for bag_filename in os.listdir(bags_dir):
        if bag_filename[-4:] != '.bag':
          continue
        if 'hand-pose' in bag_filename:
          continue

        object_name = '_'.join(bag_filename.split('_')[:-1])
        if object_names is not None:
          if object_name not in object_names:
            continue
        elif object_name in ignored_objects:
          continue
        if object_name in bags:
          bags[object_name].append(bag_filename)
        else:
          bags[object_name] = [bag_filename]

      for object_name, bag_filenames in bags.items():
        make_dir(base_dir, object_name, bag_filenames[0], count=0,
          re_extract=re_extract)
        for count, bag_filename in enumerate(bag_filenames[1:]):
          make_dir(base_dir, object_name, bag_filename, count=count+1,
            re_extract=re_extract)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--session_nums', default=None, help='comma separated')
  parser.add_argument('--object_names', default=None, help='comma separated')
  parser.add_argument('--instructions', default=None, help='comma separated')
  parser.add_argument('--re_extract', action='store_true')
  args = parser.parse_args()

  create_dirs(args.instructions, args.session_nums, args.object_names,
    args.re_extract)

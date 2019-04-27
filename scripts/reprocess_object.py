"""
Script for re-extracting contents of bag file, processing the object and then
deleting the pointclouds
Can work for multiple objects and sessions
"""
import os
import argparse
from subprocess import Popen
from dataset_utils import use_data_dirs, handoff_data_dirs
from process_session import process_session
from delete_pointclouds import delete_object_pointclouds
from IPython.core.debugger import set_trace
osp = os.path


def process_object(object_names, session_nums, instruction,
    re_extract=True, delete_pointclouds=False, play_bag=True):
  data_dirs = use_data_dirs if instruction == 'use' else handoff_data_dirs
  for session_num in session_nums:
    session_name = 'full{:d}_{:s}'.format(session_num, instruction)
    data_dir = data_dirs[session_num-1]

    if object_names is None:
      ons = next(os.walk(osp.join(data_dir, session_name)))[1]
    else:
      ons = object_names[:]

    for object_name in ons:
      base_dir = osp.join(data_dir, session_name, object_name)
      if not osp.isfile(osp.join(base_dir, 'object_name.txt')):
        continue
      if delete_pointclouds:
        delete_object_pointclouds(base_dir)
      else:
        if play_bag:
          # play the bag file
          # determine if the session was recorded with the old system
          old = session_num <= 4
          cmd = 'roslaunch contactdb_utils play_bag.launch data_dir:={:s} ' \
                'object_name:={:s} p_id:={:s}'.format(data_dir, object_name,
                    session_name)
          if old:
            cmd += ' old:=true'
          if re_extract:
            cmd += ' re_extract:=true'
          Popen(cmd, shell=True).wait()

        # process the data
        process_session(session_name, data_dir, include_objects=[object_name])


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_names', default=None, help='comma separated')
  parser.add_argument('--instruction', required=True)
  parser.add_argument('--session_nums', default=None, help='comma-separated')
  parser.add_argument('--no_play_bag', action='store_false', dest='play_bag')
  parser.add_argument('--no_re_extract', action='store_false', dest='re_extract')
  parser.add_argument('--delete_pointclouds', action='store_true')
  args = parser.parse_args()

  session_nums = args.session_nums
  if session_nums is not None:
    if '-' in session_nums:
      start, end = session_nums.split('-')
      session_nums = list(range(int(start), int(end)+1))
    else:
      session_nums = [int(n) for n in args.session_nums.split(',')]
  else:
    session_nums = list(range(51))

  object_names = args.object_names
  if object_names is not None:
    object_names = object_names.split(',')

  process_object(object_names, session_nums, args.instruction, args.re_extract,
      args.delete_pointclouds, args.play_bag)

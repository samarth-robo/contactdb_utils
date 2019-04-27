"""
script to redo the texture mapping for an entire session
"""
import sys
import os
import argparse
from texture_mapping import map_texture
from Logger import Logger
osp = os.path

ignore_objects = ['usb_drive', 'wrench']

# useful for re-processing some particular objects
# include_objects = ['pyramid_small', 'pyramid_medium', 'pyramid_large']

def process_session(session_name, data_dir, models_dir):
  session_dir = osp.join(data_dir,session_name)

  # texture map
  for object_name in sorted(os.listdir(session_dir)):
    object_dir = osp.join(session_dir, object_name)
    real_object_name = object_name.split('-')[0]
    kwargs = {}
    # if real_object_name not in include_objects:
    #   continue
    if not osp.isdir(object_dir):
      continue
    try:
      map_texture(object_name, session_name, data_dir, models_dir, **kwargs)
    except:
      print('#### ERROR IN TEXTURE MAPPING FOR {:s} @@@@'.format(object_dir))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dir', help='Data Directory',
    default=osp.join('..', 'data', 'contactdb_data'))
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--models_dir', help='Models Directory',
    default=osp.join('..', 'data', 'contactdb_3d_models'))
  args = parser.parse_args()

  log_filename = osp.join(args.data_dir, args.session_name,
  'texture_mapping_log.txt')
  log_filename = osp.expanduser(log_filename)
  print('Logging to {:s}'.format(log_filename))
  stdout = Logger(log_filename)
  sys.stdout = stdout

  process_session(session_name=args.session_name,
    data_dir=osp.expanduser(args.data_dir),
    models_dir=osp.expanduser(args.models_dir))

  stdout.delink()

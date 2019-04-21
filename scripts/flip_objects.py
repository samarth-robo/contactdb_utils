"""
Modifies the object_flip.txt for all objects in a session
"""
import argparse
import os
import numpy as np
osp = os.path


def flip_objects(data_dir, dx, dy, dz, rx, ry, rz):
  for dir_name in os.listdir(data_dir):
    dir_name = osp.join(data_dir, dir_name)
    if not osp.isdir(dir_name):
      continue
    if not osp.isfile(osp.join(dir_name, 'object_name.txt')):
      continue

    flip_filename = osp.join(dir_name, 'object_flip.txt')
    try:
      odx, ody, odz, orx, ory, orz = np.loadtxt(flip_filename, dtype=int)
    except IOError:
      pass
    
    flip_string = ' '.join(['{:d}']*6)
    flip_string = flip_string.format(odx+dx, ody+dy, odz+dz, orx+rx, ory+ry,
        orz+rz)

    with open(flip_filename, 'w') as f:
      f.write('{:s}\n'.format(flip_string))

    print('Processed {:s}'.format(dir_name))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--dx', type=int, default=0)
  parser.add_argument('--dy', type=int, default=0)
  parser.add_argument('--dz', type=int, default=0)
  parser.add_argument('--rx', type=int, default=0)
  parser.add_argument('--ry', type=int, default=0)
  parser.add_argument('--rz', type=int, default=0)
  parser.add_argument('--data_dir', required=True)
  args = parser.parse_args()

  flip_objects(osp.expanduser(args.data_dir), args.dx, args.dy, args.dz,
    args.rx, args.ry, args.rz)

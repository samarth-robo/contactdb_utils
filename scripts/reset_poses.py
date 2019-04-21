import argparse
import os
import numpy as np
osp = os.path

def reset_poses(base_dir, pose=None, verbose=False):
  base_dir = osp.join(base_dir, 'poses')
  for filename in os.listdir(base_dir):
    filename = osp.join(base_dir, filename)

    if '.txt' not in filename:
      continue

    if pose is not None:
      if pose not in filename:
        continue

    if 'tt_' not in filename:
      os.remove(filename)

    if 'tt_frame_' not in filename:
      continue

    d = np.loadtxt(filename)
    if d.ndim > 1:
      d = d[0]
    np.savetxt(filename, d[np.newaxis, :])
  if verbose:
    msg = 'Reset poses for {:s}'.format(base_dir)
    if pose is not None:
      msg = '{:s}:{:s}'.format(msg, pose)
    print(msg)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name', required=True)
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--data_dir',
    default=osp.join('~', 'deepgrasp_data', 'data'))
  parser.add_argument('--pose', default=None)
  args = parser.parse_args()

  base_dir = osp.join(args.data_dir, args.session_name, args.object_name)
  base_dir = osp.expanduser(base_dir)

  reset_poses(base_dir, pose=args.pose, verbose=True)

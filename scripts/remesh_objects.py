"""
This script remeshes all STL files and saves them as PLY
"""
import os
from subprocess import Popen

osp = os.path

def remesh(data_dir):
  data_dir = osp.expanduser(data_dir)

  for stl_filename in os.listdir(data_dir):
    if '.stl' not in stl_filename:
      continue
    print 'Processing ', stl_filename

    ply_filename = stl_filename.split('.')[0] + '.ply'
    stl_filename = osp.join(data_dir, stl_filename)
    ply_filename = osp.join(data_dir, ply_filename)

    # call Meshlab and do scaling
    script_file = osp.abspath(osp.join('.', 'remesh.mlx'))
    Popen('meshlabserver -i {:s} -o {:s} -s {:s}'.
      format(stl_filename, ply_filename, script_file), shell=True).wait()


if __name__ == '__main__':
  data_dir = osp.join('~', 'deepgrasp_data_devel', 'models')
  remesh(data_dir)
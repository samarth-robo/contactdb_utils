"""
Python script that uses meshlabserver to scale the object mesh
"""
import argparse
import os.path as osp
import xml.etree.ElementTree as ET
from subprocess import Popen
import tempfile

if __name__ == '__main__':
  parser = argparse.ArgumentParser('Mesh scaling script')
  parser.add_argument('--data_dir', type=str, help='Root data directory',
                      default=osp.expanduser(osp.join('~', 'deepgrasp_data')))
  parser.add_argument('--p_id', type=str, help='ID of participant',
    required=True)
  parser.add_argument('--object_name', type=str, help='Name of object',
                      required=True)
  args = parser.parse_args()

  base_dir = osp.join(osp.expanduser(args.data_dir), args.p_id,
    args.object_name)
  object_name = args.object_name.split('-')[0]
  filename = osp.join(base_dir, 'scale.txt')
  with open(filename, 'r') as f:
    scale = float(next(f).rstrip())
  print 'Scaling {:s} by {:6.5f}'.format(object_name, scale)

  # generate MLX script
  et = ET.parse('scaling_script_template.mlx')
  p = next(et.iter('Param'))
  p.set('value', '{:6.5f}'.format(scale))
  with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.mlx') as\
      script_file:
    et.write(script_file)

  # call Meshlab and do scaling
  input_filename = osp.join(args.data_dir, 'models',
    '{:s}.ply'.format(object_name))
  output_filename = osp.join(base_dir, 'object_scaled.ply')
  Popen('meshlabserver -i {:s} -o {:s} -s {:s}'.
        format(input_filename, output_filename, script_file.name), shell=True).\
    wait()
  print 'Scaling done'
import os
import argparse
import open3d
import matplotlib.pyplot as plt
import numpy as np
import dataset_utils
osp = os.path


def texture_proc(colors, a=0.05, invert=False):
  idx = colors > 0
  ci = colors[idx]
  if len(ci) == 0:
    return colors
  if invert:
    ci = 1 - ci
  # fit a sigmoid
  x1 = min(ci); y1 = a
  x2 = max(ci); y2 = 1-a
  lna = np.log((1 - y1) / y1)
  lnb = np.log((1 - y2) / y2)
  k = (lnb - lna) / (x1 - x2)
  mu = (x2*lna - x1*lnb) / (lna - lnb)
  # apply the sigmoid
  ci = np.exp(k * (ci-mu)) / (1 + np.exp(k * (ci-mu)))
  colors[idx] = ci
  colors = plt.cm.inferno(colors)[:, :3]
  return colors


def show_object_mesh(base_dir, filename_suffix='', sigmoid_a=0.05,
    output_filename='test.png'):
  object_name_filename = osp.join(base_dir, 'object_name.txt')
  with open(object_name_filename, 'r') as f:
    object_name = f.readline().strip()

  mesh_filename = osp.join(base_dir, 'thermal_images',
      '{:s}_textured{:s}.ply'.format(object_name, filename_suffix))
  m = open3d.read_triangle_mesh(mesh_filename)
  if not m.has_vertex_normals():
    m.compute_vertex_normals()
  m.compute_triangle_normals()

  # apply colormap
  colors = np.asarray(m.vertex_colors)[:, 0]
  colors = texture_proc(colors, a=sigmoid_a, invert=('full14' in mesh_filename))
  m.vertex_colors = open3d.Vector3dVector(colors)

  show_object_mesh.im_count = 0
  show_object_mesh.output_filename_template = '{:s}_{:s}.png'.\
      format(output_filename.split('.')[0], '{:02d}')
  def imsave_cb(vis):
    glb = show_object_mesh
    im = vis.capture_screen_float_buffer(False)
    im_filename = glb.output_filename_template.format(glb.im_count)
    plt.imsave(im_filename, np.asarray(im), dpi=1)
    print('Saved {:s}'.format(im_filename))
    glb.im_count = glb.im_count + 1
    return False

  open3d.draw_geometries_with_key_callbacks([m], {ord("."): imsave_cb})


def show_all_object_meshes(base_dir, filename_suffix='', sigmoid_a=0.05):
  for object_name in os.listdir(base_dir):
    if object_name == 'bags':
      continue
    show_object_mesh(osp.join(base_dir, object_name), filename_suffix,
        sigmoid_k)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name')
  parser.add_argument('--session_number', required=True)
  parser.add_argument('--instruction', required=True)
  parser.add_argument('--suffix', default='')
  parser.add_argument('--sigmoid_a', default=0.05)
  args = parser.parse_args()

  data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(args.instruction))
  session_name = 'full{:s}_{:s}'.format(args.session_number, args.instruction)
  base_dir = osp.join(data_dirs[int(args.session_number)-1], session_name)
  sigmoid_a = float(args.sigmoid_a)

  if args.object_name is not None:
    base_dir = osp.join(base_dir, args.object_name)
    show_object_mesh(base_dir, args.suffix, sigmoid_a)
  else:
    show_all_object_meshes(base_dir, args.suffix, sigmoid_a)

import os
import argparse
import open3d
import matplotlib.pyplot as plt
import numpy as np
from IPython.core.debugger import set_trace
osp = os.path

def show_object_mesh(base_dir, filename_suffix='', sigmoid_k=-1, max_frac=0.75,
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

  # sigmoid
  if sigmoid_k > 0:
    # decide threshold
    idx = colors > 0
    ci = colors[idx]
    thresh = max_frac * np.max(ci)
    ci = np.exp(sigmoid_k * (ci-thresh)) / (1 + np.exp(sigmoid_k * (ci-thresh)))
    colors[idx] = ci

  colors = plt.cm.inferno(colors)[:, :3]

  # fn = '/home/samarth/research/contact_heatmaps_ml/data/palm_prints/palm_print_11_handoff.ply'
  # mm = open3d.read_triangle_mesh(fn)
  # cc = np.asarray(mm.vertex_colors)[:, 0]
  # idx = cc > 0
  # ci = cc[idx]
  # thresh = max_frac * np.max(ci)
  # ci = np.exp(sigmoid_k * (ci-thresh)) / (1 + np.exp(sigmoid_k * (ci-thresh)))
  # idx = np.where(idx)[0][ci > 0.4]
  # cc = np.vstack((np.zeros(len(idx)), np.ones(len(idx)), np.zeros(len(idx)))).T

  # tree = open3d.KDTreeFlann(m)
  # idxs = []
  # for pp in np.asarray(mm.vertices)[idx]:
  #   _, i, _ = tree.search_knn_vector_3d(pp, 1)
  #   idxs.append(i[0])
  # colors[idxs] = cc

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


def show_all_object_meshes(base_dir, filename_suffix='', sigmoid_k=-1):
  for object_name in os.listdir(base_dir):
    if object_name == 'bags':
      continue
    show_object_mesh(osp.join(base_dir, object_name), filename_suffix,
        sigmoid_k)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--object_name')
  parser.add_argument('--session_name', required=True)
  parser.add_argument('--data_dir',
    default=osp.join('~', 'deepgrasp_data', 'data'))
  parser.add_argument('--suffix', default='')
  parser.add_argument('--sigmoid_k', default=10.0)
  args = parser.parse_args()

  base_dir = osp.join(osp.expanduser(args.data_dir), args.session_name)
  sigmoid_k = float(args.sigmoid_k)

  if args.object_name is not None:
    base_dir = osp.join(base_dir, args.object_name)
    show_object_mesh(base_dir, args.suffix, sigmoid_k)
  else:
    show_all_object_meshes(base_dir, args.suffix, sigmoid_k)

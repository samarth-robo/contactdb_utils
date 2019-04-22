"""
This scripts takes the output of the ICP-GUI and generates the camera poses
necessary for texturing the scaled mesh
Notation: T_a_b represents the pose of frame b w.r.t. frame a
"""
import os
import argparse
import numpy as np
from glob import glob
from tf import transformations as t
import matplotlib.pyplot as plt
import pickle
from fit_circle import fit_circle_3d
import sys
import shutil
osp = os.path
from IPython.core.debugger import set_trace

def average_quaternions(qs, ws=None):
  """
  Averages the input quaternions using SLERP
  :param qs: list of input quaternions
  :param ws: weights
  :return: average quaternion
  """
  if ws is None:
    ws = np.ones(len(qs)) / len(qs)
  else:
    assert sum(ws) == 1

  for i in xrange(1, len(qs)):
    frac = ws[i] / (ws[i-1] + ws[i])  # weight of qs[i]
    qs[i] = t.quaternion_slerp(qs[i-1], qs[i], fraction=frac)
    ws[i] = 1 - sum(ws[i+1:])

  return qs[-1]

def rotmat_from_vecs(v1, v2=np.asarray([0, 0, 1])):
  """
  Returns a rotation matrix R_1_2
  :param v1: vector in frame 1
  :param v2: vector in frame 2
  :return:
  """
  v1 = v1 / np.linalg.norm(v1)
  v2 = v2 / np.linalg.norm(v2)
  v = np.cross(v2, v1)
  vx = np.asarray([
    [0,    -v[2], +v[1], 0],
    [+v[2], 0,    -v[0], 0],
    [-v[1], +v[0], 0,    0],
    [0,     0,     0,    0]])
  dotp = np.dot(v1, v2)

  return np.eye(4) + vx + np.dot(vx, vx)/(1+dotp)


class PoseGenerator(object):
  """
  This class generates object poses for all views, interpolating whenver
  necessary
  """
  def __init__(self, data_dir):
    self.data_dir = data_dir

    # read the pose of Boson w.r.t. Kinect
    pickle_filename = osp.join('..', 'calibrations', 'stereo.pkl')
    with open(pickle_filename, 'rb') as f:
      d = pickle.load(f)
    T_k_b = np.eye(4)
    T_k_b[:3, :3] = d['R']
    T_k_b[:3, 3:] = d['T']
    self.T_k_b = T_k_b

  def estimate_poses(self, object_name, p_id, ignore_gt, symmetric,
      anchor_view=None, debug_mode=False):
    if symmetric:
      ignore_gt = True
      if debug_mode:
        print('Object is symmetric, setting ignore_gt to True')
    base_dir = osp.join(osp.expanduser(self.data_dir), p_id, object_name)

    # read object flipping information
    tx, ty, tz, rx, ry, rz = np.loadtxt(osp.join(base_dir, 'object_flip.txt'))
    T_flip = t.euler_matrix(np.deg2rad(rx), np.deg2rad(ry), np.deg2rad(rz))
    T_flip[0, 3] = tx / 100.0
    T_flip[1, 3] = ty / 100.0
    T_flip[2, 3] = tz / 100.0

    # read all T_camera_object's
    pose_filenames = sorted(glob(osp.join(base_dir, 'poses', 'tt_frame_*.txt')))
    T_c_os = []
    T_ttb_ttfs = []
    icp_filenames = []
    anchor_idx = 0
    for pose_filename in pose_filenames:
      p = np.loadtxt(pose_filename)
      view_name = pose_filename.split('.')[0].split('/')[-1].split('_')[-1]
      if p.ndim == 1:
        continue
      else:
        T_ttb_ttf = np.eye(4)
        T_ttb_ttf[:3, 3]  = p[0, :3]
        T_ttb_ttf[:3, :3] = p[0, 3:].reshape((3, 3))
        T_ttb_ttfs.append(T_ttb_ttf)
        T_c_o = np.eye(4)
        T_c_o[:3, 3]  = p[1, :3]
        T_c_o[:3, :3] = p[1, 3:].reshape((3, 3))
        T_c_os.append(T_c_o)
        icp_filenames.append(pose_filename)
        if view_name == anchor_view:
          anchor_idx = len(T_c_os)-1

    if len(T_c_os) >= 3:
      # construct T_c_ttb: translation from centre of the fitted circle,
      # orientation from the circle plane estimate
      object_centres = np.asarray([t.translation_from_matrix(np.dot(tco, T_flip))
        for tco in T_c_os])
      tt_c, tt_r, tt_n = fit_circle_3d(object_centres)
      # set_trace()
      # flip normal if it does not point upwards (-Y direction)
      if np.dot(tt_n, [0, -1, 0]) < 0:
        tt_n *= -1
      # alternative: T_c_ttb from the PE plane estimate
      # filename = osp.join(base_dir, 'poses', 'tt_base_processed.txt')
      # p = np.genfromtxt(filename, skip_footer=1)
      # T_c_ttb = np.eye(4)
      # T_c_ttb[:3, :3] = p[3:].reshape((3, 3))
      if symmetric and (tt_r <= 3e-2):  # object is placed at center of TT
        tt_c = np.mean(object_centres, axis=0)
        z_axis = np.asarray([0,0,1])[:, np.newaxis]
        tt_n_bar = np.hstack([np.dot(tco[:3, :3], z_axis) for tco in T_c_os]).T
        if symmetric:
          tt_n_bar = tt_n_bar[anchor_idx:anchor_idx+1]
        tt_n = np.mean(tt_n_bar, axis=0)
      T_c_ttb = rotmat_from_vecs(tt_n)
      T_c_ttb[:3, 3] = tt_c

      # look-up T_ttf_o
      T_ttf_os = []
      for idx, (T_ttb_ttf, T_c_o) in enumerate(zip(T_ttb_ttfs, T_c_os)):
        T_c_ttf = np.dot(T_c_ttb, T_ttb_ttf)
        T_ttf_o = np.dot(t.inverse_matrix(T_c_ttf), T_c_o)
        T_ttf_os.append(T_ttf_o)
      # if symmetric:
      #   T_ttf_os = [T_ttf_os[anchor_idx]]
      # average out the T_ttf_os
      # by mean
      # q_bar = average_quaternions([t.quaternion_from_matrix(T_ttf_o)
      #   for T_ttf_o in T_ttf_os])
      # r_bar = t.euler_from_quaternion(q_bar)
      # by median
      r_bar = np.asarray([t.euler_from_matrix(T_ttf_o) for T_ttf_o in T_ttf_os])
      # fix angle flips
      r_bar[1:] += (np.abs(r_bar[1:]-r_bar[:1]) > np.pi) * np.sign(r_bar[:1]) * 2*np.pi
      r_bar = np.mean(r_bar, axis=0)
      t_bar = np.mean([t.translation_from_matrix(T_ttf_o) for T_ttf_o in T_ttf_os],
        axis=0)
      # zero out-of-plane rotations and translations
      # r_bar[0] = 0
      # r_bar[1] = 0
      # t_bar[2] = 0
      T_ttf_o = t.euler_matrix(*r_bar)
      T_ttf_o[:3, 3] = t_bar
    else:  # use only the ICP poses
      T_c_ttb = T_ttf_o = np.eye(4)
      pose_filenames = icp_filenames
      if debug_mode:
        print('Using only ICP poses')

    # read camera info
    filename = osp.join(base_dir, 'thermal_images', 'camera_info.txt')
    fx, fy, cx, cy, width, height = np.loadtxt(filename)

    # generate T_o_c for all pose files and save pose files
    ps_gt = []
    ps_pred = []
    for pose_filename in pose_filenames:
      p = np.loadtxt(pose_filename)
      if p.ndim == 1:
        p = p[np.newaxis, :]

      T_ttb_ttf = np.eye(4)
      T_ttb_ttf[:3, 3]  = p[0, :3]
      T_ttb_ttf[:3, :3] = p[0, 3:].reshape((3, 3))
      T_c_ttf = np.dot(T_c_ttb, T_ttb_ttf)
      T_c_o = np.dot(T_c_ttf, T_ttf_o)

      if len(p) > 1:
        T = np.eye(4)
        T[:3, 3]  = p[1, :3]
        T[:3, :3] = p[1, 3:].reshape((3, 3))
        if not ignore_gt:
          T_c_o = T
        # else:  # use if turntable angles are erroneous
        #   T_c_o[:3, 3] = T[:3, 3]
        ps_gt.append(t.translation_from_matrix(T))
      ps_pred.append(t.translation_from_matrix(T_c_o))

      # un-flip the object
      T_c_o = np.dot(T_c_o, T_flip)

      count = pose_filename.split('.')[0].split('/')[-1].split('_')[-1]

      # pose of object w.r.t Kinect, for the check_camera_poses utility
      filename = osp.join(base_dir, 'poses', 'object_pose_{:s}.txt'.format(count))
      with open(filename, 'w') as f:
        f.write('# translations\n')
        f.write('{:7.6f}\n{:7.6f}\n{:7.6f}\n\n'.
                format(*t.translation_from_matrix(T_c_o)))
        f.write('# rotations\n')
        f.write('{:7.6f} {:7.6f} {:7.6f}\n{:7.6f} {:7.6f} {:7.6f}\n'
                '{:7.6f} {:7.6f} {:7.6f}\n\n'.format(*(T_c_o[:3, :3].flatten())))
        f.write('# camera params\n')
        f.write('{:7.6f}\n'.format(fx))
        f.write('{:7.6f}\n'.format(fy))
        f.write('{:7.6f}\n'.format(cx))
        f.write('{:7.6f}\n'.format(cy))
        f.write('{:d}\n'.format(int(width)))
        f.write('{:d}\n'.format(int(height)))

      # pose w.r.t Boson T_b_o = T_b_c * T_c_o
      T_c_o = np.dot(t.inverse_matrix(self.T_k_b), T_c_o)
      T_o_c = t.inverse_matrix(T_c_o)

      # pose of Boson w.r.t object, for Open3D texture mapping
      filename = osp.join(base_dir, 'poses', 'camera_pose_{:s}.txt'.format(count))
      with open(filename, 'w') as f:
        f.write('# translations\n')
        f.write('{:7.6f}\n{:7.6f}\n{:7.6f}\n\n'.
                format(*t.translation_from_matrix(T_o_c)))
        f.write('# rotations\n')
        f.write('{:7.6f} {:7.6f} {:7.6f}\n{:7.6f} {:7.6f} {:7.6f}\n'
                '{:7.6f} {:7.6f} {:7.6f}\n\n'.format(*(T_o_c[:3, :3].flatten())))
        f.write('# camera params\n')
        f.write('{:7.6f}\n'.format(fx))
        f.write('{:7.6f}\n'.format(fy))
        f.write('{:7.6f}\n'.format(cx))
        f.write('{:7.6f}\n'.format(cy))
        f.write('{:d}\n'.format(int(width)))
        f.write('{:d}\n'.format(int(height)))

      # pose of object w.r.t. Boson, for seamless texture mapping using 
      # https://github.com/nmoehrle/mvs-texturing (deprecated)
      filename = osp.join(base_dir, 'thermal_images', '{:s}.cam'.format(count))
      with open(filename, 'w') as f:
        f.write('{:7.6f} {:7.6f} {:7.6f} '.
                format(*t.translation_from_matrix(T_c_o)))
        f.write('{:7.6f} {:7.6f} {:7.6f} {:7.6f} {:7.6f} {:7.6f} '
                '{:7.6f} {:7.6f} {:7.6f}\n'.format(*(T_c_o[:3, :3].flatten())))
        f.write('{:7.6f} 0 0 '.format(fx / width))
        f.write('{:7.6f} '.format(fy / fx))
        f.write('{:7.6f} {:7.6f}'.format(cx/width, cy/height))

    if len(ps_gt) < 3 or not debug_mode:
      return

    ps_gt = np.asarray(ps_gt)
    ps_pred = np.asarray(ps_pred)

    c_gt, _, _ = fit_circle_3d(ps_gt)
    c_pred, _, _ = fit_circle_3d(ps_pred)

    ps_gt = np.vstack((ps_gt, c_gt))
    ps_pred = np.vstack((ps_pred, c_pred))

    print 'Pred center = ', c_pred
    print 'GT center = ', c_gt

    ps_gt = np.vstack((ps_gt.T, np.ones(len(ps_gt))))
    ps_gt = np.dot(t.inverse_matrix(T_c_ttb), ps_gt)
    ps_gt = ps_gt[:3].T
    ps_pred = np.vstack((ps_pred.T, np.ones(len(ps_pred))))
    ps_pred = np.dot(t.inverse_matrix(T_c_ttb), ps_pred)
    ps_pred = ps_pred[:3].T
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(ps_pred[:, 0], ps_pred[:, 1], c='r')
    # for i, txt in enumerate(xrange(len(ps_pred))):
    #   ax.annotate(txt, (ps_pred[i, 0], ps_pred[i, 1]))
    ax.scatter(ps_gt[:, 0], ps_gt[:, 1], c='g')
    for i, txt in enumerate(xrange(len(ps_gt))):
      ax.annotate(txt, (ps_gt[i, 0], ps_gt[i, 1]))
    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
  parser = argparse.ArgumentParser('Mesh scaling script')
  parser.add_argument('--data_dir', help='Root data directory',
    default=osp.expanduser(osp.join('~', 'deepgrasp_data', 'data')))
  parser.add_argument('--session_name', help='ID of participant',
    required=True)
  parser.add_argument('--object_name', help='Name of object',
                      required=True)
  parser.add_argument('--ignore_gt', action='store_true',
    help='If used, this flag uses GT poses to fit the circle, but uses turntable'
         'rotation to get final poses')
  parser.add_argument('--symmetric', action='store_true',
    help='Flag for rotationally symmetric objects')
  parser.add_argument('--anchor_view',
    help='Name of the view to be used as anchor (only used for symmetric objects)')
  args = parser.parse_args()

  pg = PoseGenerator(data_dir=args.data_dir)
  pg.estimate_poses(args.object_name, args.session_name, args.ignore_gt, args.symmetric,
    args.anchor_view, debug_mode=True)

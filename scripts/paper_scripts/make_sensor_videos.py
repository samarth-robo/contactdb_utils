import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
from IPython.core.debugger import set_trace
osp = os.path


def proc_thermal(im, k=5.0, max_frac=0.75):
  global thresh
  im = im / 255.0
  if thresh < 0:
    thresh = max_frac * np.max(im)
  im = np.exp(k * (im-thresh)) / (1 + np.exp(k * (im-thresh)))
  im = plt.cm.inferno(im)[:, :, [2,1,0]]
  im = np.uint8(im*255)
  return im


def proc_depth(im):
  y, x = np.nonzero(im)
  d = im[y, x].astype(np.float)
  d -= d.min()
  d /= d.max()
  d_proc = plt.cm.viridis(d)[:, :3]
  im_proc = np.zeros((im.shape[0], im.shape[1], 3))
  im_proc[y, x, :] = d_proc
  im_proc = np.uint8(im_proc*255)
  return im_proc


def generate_videos(session_name, object_name, data_dir):
  fourcc = cv2.VideoWriter_fourcc(*'XVID')
  frame_rate = 10.0

  # RGB video
  video_size = (960, 540)
  output_filename = '{:s}_{:s}_rgb.avi'.format(object_name, session_name)
  v_writer = cv2.VideoWriter(output_filename, fourcc, frame_rate, video_size)
  for filename in sorted(next(os.walk(data_dir))[-1]):
    if 'rgb' not in filename:
      continue
    filename = osp.join(data_dir, filename)
    im = cv2.imread(filename)
    im = cv2.resize(im, video_size)
    v_writer.write(im)
  v_writer.release()
  print('{:s} written'.format(output_filename))

  # thermal video
  video_size = (640, 512)
  output_filename = '{:s}_{:s}_thermal.avi'.format(object_name, session_name)
  v_writer = cv2.VideoWriter(output_filename, fourcc, frame_rate, video_size)
  for filename in sorted(next(os.walk(data_dir))[-1]):
    if 'thermal' not in filename:
      continue
    filename = osp.join(data_dir, filename)
    im = cv2.imread(filename, -1)
    im = cv2.resize(im, video_size)
    im = proc_thermal(im)
    v_writer.write(im)
  v_writer.release()
  print('{:s} written'.format(output_filename))

  # depth video
  video_size = (640, 512)
  output_filename = '{:s}_{:s}_depth.avi'.format(object_name, session_name)
  v_writer = cv2.VideoWriter(output_filename, fourcc, frame_rate, video_size)
  for filename in sorted(next(os.walk(data_dir))[-1]):
    if 'depth' not in filename:
      continue
    filename = osp.join(data_dir, filename)
    im = cv2.imread(filename, -1)
    im = cv2.resize(im, video_size)
    im = proc_depth(im)
    v_writer.write(im)
  v_writer.release()
  print('{:s} written'.format(output_filename))


if __name__ == '__main__':
  session_name = 'full11_use'
  object_name = 'water_bottle'
  data_dir = osp.join('~', 'deepgrasp_data', 'data', session_name,
      object_name, 'sync_images')
  data_dir = osp.expanduser(data_dir)

  global thresh
  thresh = -1
  generate_videos(session_name, object_name, data_dir)

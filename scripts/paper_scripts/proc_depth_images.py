import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
from IPython.core.debugger import set_trace
osp = os.path

S = 300
X = 150
Y = 270
dir_name = 'depth_images'

def process():
  im_filenames = [fn for fn in next(os.walk(dir_name))[-1] if '.png' in fn]
  for im_filename in sorted(im_filenames):
    if '.png' not in im_filename:
      continue
    output_filename = '{:s}_depth_proc.png'.format(im_filename.split('.')[0])
    im_filename = osp.join(dir_name, im_filename)
    print('Read {:s}'.format(im_filename))
    im = cv2.imread(im_filename, -1)
    im = im[Y:Y+S, X:X+S].astype(float)

    y,x = np.nonzero(im)
    d = im[y, x]
    d -= d.min()
    d /= d.max()
    d_proc = plt.cm.viridis(d)[:, :3]

    im_proc = np.zeros((im.shape[0], im.shape[1], 3))
    im_proc[y, x, :] = d_proc
    
    cv2.imwrite(output_filename, (im_proc*255).astype(np.uint8)) 

if __name__ == '__main__':
  process()

import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
from IPython.core.debugger import set_trace
osp = os.path

S = 70
X = 100
Y = 140
sigmoid_k = 10
max_frac = 0.75

def process():
  im_filenames = [fn for fn in next(os.walk('thermal_images'))[-1] if '.png' in fn]
  for im_filename in sorted(im_filenames):
    if '.png' not in im_filename:
      continue
    output_filename = '{:s}_thermal_proc.png'.format(im_filename.split('.')[0])
    im_filename = osp.join('thermal_images', im_filename)
    print('Read {:s}'.format(im_filename))
    im = cv2.imread(im_filename, -1)
    if im.ndim == 3:
      im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    im = im[Y:Y+S, X:X+S] / 255.0

    thresh = max_frac * np.max(im)
    im = np.exp(sigmoid_k * (im-thresh)) / (1 + np.exp(sigmoid_k * (im-thresh)))

    im_proc = plt.cm.inferno(im)[:, :, [2,1,0]]
    
    cv2.imwrite(output_filename, (im_proc*255).astype(np.uint8)) 

if __name__ == '__main__':
  process()

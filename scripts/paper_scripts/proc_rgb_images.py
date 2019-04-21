import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
from IPython.core.debugger import set_trace
osp = os.path

S = 170
X = 350
Y = 250
dir_name = 'rgb_images'

def process():
  im_filenames = [fn for fn in next(os.walk(dir_name))[-1] if '.png' in fn]
  for im_filename in sorted(im_filenames):
    if '.png' not in im_filename:
      continue
    output_filename = '{:s}_rgb_proc.png'.format(im_filename.split('.')[0])
    im_filename = osp.join(dir_name, im_filename)
    print('Read {:s}'.format(im_filename))
    im = cv2.imread(im_filename, -1)
    im = im[Y:Y+S, X:X+S]
    cv2.imwrite(output_filename, im)

if __name__ == '__main__':
  process()

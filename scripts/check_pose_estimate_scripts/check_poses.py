import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from IPython.core.debugger import set_trace
osp = os.path

from scipy.misc import imread


base_path = osp.expanduser('./cropped_handoff')
dirs = ['train']

try:
    f = open('wrong_masks.txt', 'r')
    lines = f.readlines()
    # last_obj = lines[-2][:-1]
    last_obj = lines[-1][:-1]
    f.close()
except:
    last_obj = None

# last_obj = 'wine_glass_full32_handoff_07.png'

outf = open('wrong_masks.txt', 'a')

plt.ion()
fig, ax = plt.subplots(1,3,figsize=(12,5))
fig_count = 0

for d in dirs:
    img_names = list(sorted(os.listdir(os.path.join(base_path, 'A', d))))
    if last_obj:
        idx = img_names.index(last_obj)
        img_names = img_names[idx+1:]

    i = 0
    while i < len(img_names):
        img = img_names[i]
        rgb = imread(os.path.join(base_path, 'A', d, img))
        m = imread(os.path.join(base_path, 'C', d, img))
        mask = np.stack((m,)*3, axis=-1)
        com = np.multiply(rgb, mask)

        plt.sca(ax[0])
        plt.cla()
        ax[0].imshow(rgb)
        plt.sca(ax[1])
        plt.cla()
        ax[1].imshow(com)
        plt.sca(ax[2])
        plt.cla()
        ax[2].imshow(m)
        plt.title(img)
        plt.show()
        fig_count += 1

        tmp = str(raw_input('(x) incorrect, (Enter) next, (p) previous, (q) quit: '))
        if tmp == 'x':
            outf.write(img+'\n')
            outf.flush()
            i += 1
        elif tmp == 'p':
            i -= 1
        elif tmp == 'q':
            print img
            outf.close()
            sys.exit(0)
        elif tmp == 'd':
          set_trace()
        else:
            i += 1

        if fig_count % 500 == 0:
          plt.close(fig)
          fig, ax = plt.subplots(1,3,figsize=(12,5))

outf.close()

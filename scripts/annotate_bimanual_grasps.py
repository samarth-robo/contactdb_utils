import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from IPython.core.debugger import set_trace
osp = os.path

from scipy.misc import imread


dirs = ['train', 'test']

handoff = ['alarm_clock', 'banana', 'binoculars', 'bowl', 'camera',
    'cube_large', 'cylinder_large', 'eyeglasses', 'headphones',
    'piggy_bank', 'ps_controller', 'pyramid_large', 'sphere_large',
    'stanford_bunny', 'utah_teapot']
use = ['banana', 'binoculars', 'bowl', 'camera', 'eyeglasses', 'flute', 'headphones',
       'mug', 'ps_controller', 'utah_teapot', 'water_bottle']

# !!! change both `base_path` and `task` to handoff or use !!!
base_path = '/home/samarth/deepgrasp_data/cropped_no_depth_handoff/'
task = handoff # specify handoff/use
task_str = 'handoff'

try:
    f = open('{:s}_bimanual_grasps.txt'.format(task_str), 'r')
    lines = f.readlines()
    last_session = lines[-1][:-1]
    f.close()
except:
    last_session = None
last_session = 'piggy_bank_full7_handoff'

outf = open('{:s}_bimanual_grasps.txt'.format(task_str), 'a')

plt.ion()
fig, ax = plt.subplots(2, 2, figsize=(12, 5))
fig_count = 0

for d in dirs:
    img_names = list(sorted(os.listdir(os.path.join(base_path, 'B', d))))

    # group image names
    p_img_names = []
    p_session_names = []
    for img_name in img_names:
      if '01' in img_name and any(obj in img_name for obj in task):
        g = [img_name] * 4
        g[1] = g[1].replace('01', '03')
        g[2] = g[2].replace('01', '05')
        g[3] = g[3].replace('01', '07')
        p_img_names.append(g)
        session_name = '_'.join(img_name.split('_')[:-1])
        p_session_names.append(session_name)

    if last_session:
        idx = p_session_names.index(last_session)
        p_img_names = p_img_names[idx+1:]
        p_session_names = p_session_names[idx+1:]

    i = 0
    while i < len(p_img_names):
        imgs = p_img_names[i]
        session_name = p_session_names[i]
        thermals = [imread(os.path.join(base_path, 'B', d, img)) for img in imgs]
        for j in range(len(thermals)):
          r, c = divmod(j, 2)
          plt.sca(ax[r][c])
          plt.cla()
          ax[r][c].imshow(thermals[j])
        fig.suptitle(session_name)
        plt.show()
        fig_count += 1

        tmp = str(raw_input('(b) bimanual, (Enter) next, (p) previous, (q) quit: '))
        if tmp == 'b':
            outf.write(session_name+'\n')
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
          fig, ax = plt.subplots(2, 2, figsize=(12, 5))

outf.close()

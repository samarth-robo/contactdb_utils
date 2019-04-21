import glob
import numpy as np
import os
import sys
sys.path.append('..')
import dataset_utils
osp = os.path

from scipy.misc import imread, imsave
from shutil import copyfile


rgb_src = 'rgb_images'
thermal_src = 'thermal_images'
depth_src = 'depth_images'

task = 'use'
dst = task
rgb_dst = 'A/train'
thermal_dst = 'B/train'
mask_dst = 'C/train'
data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(task))

for sd in [rgb_dst, thermal_dst, mask_dst]:
    dir_path = os.path.join(dst, sd)
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

users = range(43, 51)

outf = open('combine_rgbd_log.txt', 'w')

for user in users:
    data_dir = data_dirs[user-1]
    user = 'full{:d}_{:s}'.format(user, task)
    print user
    outf.write('---'+user+'---\n')

    # full_src = os.path.join(src, user + '_' + task)
    full_src = os.path.join(data_dir, user)
    objects = list(sorted([obj for obj in os.listdir(full_src) if obj != 'bags' and os.path.isdir(os.path.join(full_src, obj))]))

    for obj in objects:
        print obj

        obj_path = os.path.join(full_src, obj)
        rgb = os.path.join(obj_path, rgb_src)
        thermal = os.path.join(obj_path, thermal_src)

        images = list(set([f[:2] for f in os.listdir(rgb) if 'registered' in f]))
        # images = list(set([f[:2] for f in os.listdir(rgb)]))
        img_names = [img for img in images if img != 'vi']
        # files = os.listdir(sync_path)
        # images = [os.path.splitext(f)[0] for f in files if os.path.splitext(f)[0][-10:] == 'registered']
        # img_names = list(set([os.path.splitext(f)[0][:5] for f in images]))
        n_views = len(img_names)

        for num in img_names:
            old_rgb_name = num + '_registered.png'
            # old_rgb_name = num + '.png'
            old_thermal_name = num + '.png'
            old_mask_name = num + '_mask.png'
            new_name = obj + '_' + user + '_' + num + '.png'
            rgb_img_src = os.path.join(rgb, old_rgb_name)
            # rgb_img_src = os.path.join(sync_path, old_rgb_name)
            rgb_img_dst = os.path.join(dst, rgb_dst, new_name)

            rgb_img = imread(rgb_img_src)

            thermal_img_src = os.path.join(thermal, old_thermal_name)
            # thermal_img_src = os.path.join(sync_path, old_thermal_name)
            thermal_img_dst = os.path.join(dst, thermal_dst, new_name)

            mask_img_src = os.path.join(thermal, old_mask_name)
            mask_img_dst = os.path.join(dst, mask_dst, new_name)

            try:
                copyfile(mask_img_src, mask_img_dst)
                copyfile(thermal_img_src, thermal_img_dst)
                imsave(rgb_img_dst, rgb_img)
            except:
                outf.write(obj+'\n')

outf.close()

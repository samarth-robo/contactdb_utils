import numpy as np
import os
from IPython.core.debugger import set_trace

from PIL import Image


src = '.'
img_type = 'use'
dst = 'cropped_' + img_type
dirs = ['train']
folders = ['A/train', 'B/train', 'C/train', 'A/test', 'B/test', 'C/test']
old = ['l1', 'l2', 'l3', 'l4']
img_size = (640, 512)
w, h = 320, 256

outf = open('crop_log.txt', 'w')

for d in dirs:
    full_dst = os.path.join(src, dst)
    if not os.path.exists(full_dst):
        for fol in folders:
            os.makedirs(os.path.join(full_dst, fol))


    path = os.path.join(src, img_type)
    images = list(sorted(os.listdir(os.path.join(path, 'A', d))))

    i = 0

    for img_name in images:
        if i % 1000 == 0:
            print i

        rgbd_path = os.path.join(path, 'A', d, img_name)
        rgbd = Image.open(rgbd_path)

        thermal_path = os.path.join(path, 'B', d, img_name)
        thermal = Image.open(thermal_path)

        mask_path = os.path.join(path, 'C', d, img_name)
        mask = Image.open(mask_path)

        session_id = img_name.split('_')[1][-2:]

        if session_id in old:
            rgbd = rgbd.resize(img_size)
            thermal = thermal.resize(img_size)
            mask = mask.resize(img_size)

            # coords = [3*w/10, h/2, 3*w/10 + w/3, h/2 + 2*h/5] # registered / thermal (old)
            # coords = [3*w/10, 2*h/5, 3*w/10 + w/3, 2*h/5 + h/2] # registered / thermal (new)
            # coords = [3*w/10, h/3, 3*w/10 + w/3, h/3 + 2*h/5] # unregistered rgbd (old)
            # coords = [3*w/10, h/3, 3*w/10 + w/3, h/3 + h/2] # unregistered rgbd (new)

        # m = transforms.ToTensor()(mask)
        # mask_pix = np.array(np.where(m == 1))
        m = np.asarray(mask)
        mask_pix = np.where(m)
        try:
            x_min, y_min = min(mask_pix[1]), min(mask_pix[0])
            x_max, y_max = max(mask_pix[1]), max(mask_pix[0])
            cx, cy = (x_min+x_max) / 2, (y_min+y_max) / 2
        except:
            print img_name
            outf.write(img_name+'\n')

        # w, h = img.size[0], img.size[1]
        # coords = [w/4, 2*h/5, w/4+w/2, 2*h/5+h/2]
        coords = [cx-w/2, cy-h/2, cx+w/2, cy+h/2]

        cropped_rgbd = rgbd.crop(coords)
        cropped_thermal = thermal.crop(coords)
        cropped_mask = mask.crop(coords)

        cropped_rgbd_path = os.path.join(full_dst, 'A', d, img_name)
        cropped_rgbd.save(cropped_rgbd_path)

        cropped_thermal_path = os.path.join(full_dst, 'B', d, img_name)
        cropped_thermal.save(cropped_thermal_path)

        cropped_mask_path = os.path.join(full_dst, 'C', d, img_name)
        cropped_mask.save(cropped_mask_path)

        i += 1

outf.close()

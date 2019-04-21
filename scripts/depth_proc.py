import numpy as np
import sys
import os.path as osp
DEPTH_PROC_PATH = osp.expanduser('~/research/depth_completion/ip_basic/')
sys.path.append(DEPTH_PROC_PATH)
from depth_map_utils import fill_in_multiscale

def fill_holes(d, scale_to_m=1000.0, min_depth=0, max_depth=None):
  df, _ = fill_in_multiscale(np.asarray(d, dtype=float) / scale_to_m)
  df = np.clip(df, a_min=min_depth, a_max=max_depth)
  df *= scale_to_m
  return df

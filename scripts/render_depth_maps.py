import trimesh
import os
import numpy as np
osp = os.path

# suppress warnings from autolab packages
import logging
logger = logging.getLogger()
prev_level = logger.level
logger.setLevel(logging.ERROR)
from autolab_core import RigidTransform
from perception import CameraIntrinsics
import meshrender as mr
logger.setLevel(prev_level)


def render_depth_maps(mesh_filename, intrinsic, extrinsics):
  scene = mr.Scene()

  # setup camera
  ci = CameraIntrinsics(
    frame='camera',
    fx=intrinsic.get_focal_length()[0],
    fy=intrinsic.get_focal_length()[1],
    cx=intrinsic.get_principal_point()[0],
    cy=intrinsic.get_principal_point()[1],
    skew=0,
    height=intrinsic.height,
    width=intrinsic.width
  )
  cp = RigidTransform(from_frame='camera', to_frame='world')
  camera = mr.VirtualCamera(ci, cp)
  scene.camera = camera

  obj = trimesh.load_mesh(mesh_filename)
  dmaps = []
  for T in extrinsics:
    obj_pose = RigidTransform(
      rotation=T[:3, :3],
      translation=T[:3, 3],
      from_frame='obj',
      to_frame='camera'
    )
    sobj = mr.SceneObject(obj, obj_pose)
    scene.add_object('obj', sobj)
    dmap = scene.render(render_color=False)
    dmap = np.uint16(dmap * 1000.0)
    dmaps.append(dmap)
    scene.remove_object('obj')

  return np.stack(dmaps)
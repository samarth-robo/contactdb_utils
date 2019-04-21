import logging
import os
osp = os.path

base_dir = osp.expanduser(osp.join('~', 'deepgrasp_data'))
use_data_dirs = \
    [osp.join(base_dir, 'data')]*28 + \
    [osp.join(base_dir, 'data3')] + \
    [osp.join(base_dir, 'data2')]*4 + \
    [osp.join(base_dir, 'data3')]*18
handoff_data_dirs = \
    [osp.join(base_dir, 'data')]*24 + \
    [osp.join(base_dir, 'data3')]*5 +\
    [osp.join(base_dir, 'data2')]*4 + \
    [osp.join(base_dir, 'data3')]*17


def get_session_mesh_filenames(session_name, data_dir):
  logger = logging.getLogger(__name__)
  base_dir = osp.join(data_dir, session_name)

  # determine the directories from which to take meshes
  object_dirs = {}
  for object_dir in next(os.walk(base_dir))[1]:
    object_dir = osp.join(base_dir, object_dir)
    object_name_filename = osp.join(object_dir, 'object_name.txt')
    try:
      with open(object_name_filename, 'r') as f:
        object_name = f.readline().strip()
    except IOError:
      # logger.warning('Skipping {:s}'.format(object_dir))
      continue
    if object_name not in object_dirs:
      object_dirs[object_name] = [object_dir]
    else:
      object_dirs[object_name].append(object_dir)

  mesh_filenames = {}
  for oname, odirs in object_dirs.items():
    if len(odirs) == 1:
      mesh_filename = osp.join(odirs[0], 'thermal_images',
        '{:s}_textured.ply'.format(oname))
    else:  # find which one has merge.txt
      merge_dirs = [od for od in odirs if osp.isfile(osp.join(od, 'merge.txt'))]
      # determine which directory has the merged mesh
      for d in merge_dirs:
        with open(osp.join(d, 'merge.txt'), 'r') as f:
          m = f.readline()
        if len(m):
          merge_dir = d
          break
      else:
        logger.error('No directory is marked as destination for {:s} {:s}'.
            format(session_name, oname))
        raise IOError
      mesh_filename = osp.join(merge_dir, 'thermal_images',
        '{:s}_textured.ply'.format(oname))
    mesh_filenames[oname] = mesh_filename
  return mesh_filenames



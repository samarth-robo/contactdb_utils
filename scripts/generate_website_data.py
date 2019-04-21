import os
import logging
import paramiko
from scp import SCPClient
import argparse
import sys
import logging
import open3d
import numpy as np
import json
from show_textured_mesh import texture_proc
import dataset_utils
import time
from IPython.core.debugger import set_trace
osp = os.path


def transfer_meshes(instructions, sessions, include_objects, dest_host,
    dest_user, dest_dir):
  logger = logging.getLogger(__name__)
  if instructions is None:
    logger.error('No instructions specified')
    return

  for instruction in instructions:
    data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction))
    
    if sessions is None:
      sessions = ['{:d}'.format(idx+1) for idx in range(len(data_dirs))]

    with paramiko.SSHClient() as ssh:
      ssh.load_system_host_keys()
      ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
      ssh.connect(hostname=dest_host, username=dest_user)

      with SCPClient(ssh.get_transport(), socket_timeout=20) as scp:
        for session_id in sessions:
          session_name = 'full{:s}_{:s}'.format(session_id, instruction)
          logger.info('Session {:s}'.format(session_name))
          idx = int(session_id) - 1
          mesh_filenames = dataset_utils.get_session_mesh_filenames(session_name,
              data_dirs[idx])
          for object_name, mesh_filename in mesh_filenames.items():
            if include_objects is not None:
              if object_name not in include_objects:
                continue
            m = open3d.read_triangle_mesh(mesh_filename)
            m.compute_vertex_normals()
            m.compute_triangle_normals()
            colors = np.asarray(m.vertex_colors)[:, 0]
            colors = texture_proc(colors, invert=('full14' in session_name))
            m.vertex_colors = open3d.Vector3dVector(colors)
            open3d.write_triangle_mesh('/tmp/tmp_contactdb_mesh.ply', m)
            dest_filename = osp.join(dest_dir, 'meshes',
                '{:s}_{:s}.ply'.format(session_name, object_name))
            scp.put('/tmp/tmp_contactdb_mesh.ply', dest_filename)
            print('Written {:s}'.format(dest_filename))
            time.sleep(1)


def transfer_datapoints(dest_host, dest_user, dest_dir):
  logger = logging.getLogger(__name__)
  d = {}
  for instruction in ['use', 'handoff']:
    di = {}
    data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction))
    sessions = ['full{:d}_{:s}'.format(idx+1, instruction)
        for idx in range(len(data_dirs))]
    for idx, session_name in enumerate(sessions):
      mesh_filenames = dataset_utils.get_session_mesh_filenames(session_name,
          data_dirs[idx])
      for object_name in mesh_filenames.keys():
        if object_name == 'palm_print':
          continue
        if object_name in di:
          di[object_name].append(idx+1)
        else:
          di[object_name] = [idx+1]
    d[instruction] = di
  src_filename = '/tmp/datapoints.json'
  with open(src_filename, 'w') as f:
    json.dump(d, f)
  dst_filename = osp.join(dest_dir, 'datapoints.json')
      
  with paramiko.SSHClient() as ssh:
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(hostname=dest_host, username=dest_user)

    with SCPClient(ssh.get_transport(), socket_timeout=15) as scp:
      scp.put(src_filename, dst_filename)


if __name__ == '__main__':
  logging.basicConfig(level=logging.INFO)

  parser = argparse.ArgumentParser()
  parser.add_argument('--instructions', help='comma separated', default=None)
  parser.add_argument('--sessions', help='1,2,3 or 1-3 (inclusive)', default=None)
  parser.add_argument('--objects', help='comma separater', default=None)
  parser.add_argument('--dest_host', default='contactdb.cc.gatech.edu')
  parser.add_argument('--dest_user', default='contactdb')
  parser.add_argument('--dest_dir', default='httpdocs')
  args = parser.parse_args()

  sessions = args.sessions
  if sessions is not None:
    if '-' in sessions:
      start, end = sessions.split('-')
      sessions = ['{:d}'.format(s) for s in range(int(start), int(end)+1)]
    else:
      sessions = sessions.split(',')
  instructions = args.instructions
  if instructions is not None:
    instructions = instructions.split(',')
  objects = args.objects
  if objects is not None:
    objects = objects.split(',')
  transfer_meshes(instructions, sessions, objects, args.dest_host, args.dest_user,
      args.dest_dir)
  # transfer_datapoints(args.dest_host, args.dest_user, args.dest_dir)

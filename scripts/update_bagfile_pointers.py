import os
import dataset_utils
from IPython.core.debugger import set_trace
osp = os.path

def update(instruction):
  data_dirs = getattr(dataset_utils, '{:s}_data_dirs'.format(instruction))
  for session_idx, data_dir in enumerate(data_dirs):
    session_name = 'full{:d}_{:s}'.format(session_idx+1, instruction)
    data_dir = osp.join(data_dir, session_name)
    for object_dir in next(os.walk(data_dir))[1]:
      object_dir = osp.join(data_dir, object_dir)
      recording_filename = osp.join(object_dir, 'recording.txt')
      if not osp.isfile(recording_filename):
        continue
      with open(recording_filename, 'r') as f:
        line = next(f).strip()
      bag_filename = line.split('/')[-1]
      line = osp.join('..', '..', 'rosbags', session_name, bag_filename)
      with open(recording_filename, 'w') as f:
        f.write(line + '\n')
      print('{:s} updated'.format(recording_filename))


if __name__ == '__main__':
  update('use')
  update('handoff')

#!/usr/bin/env python

import roslib
package_name = 'contactdb_utils'
roslib.load_manifest(package_name)
import rospy
import tf2_ros
from tf_conversions import transformations as tx
import numpy as np
import pickle
import os.path as osp
import rospkg
from geometry_msgs.msg import TransformStamped


if __name__ == '__main__':
  rospy.init_node('thermal_tf_broadcaster')
  rospack = rospkg.RosPack()

  # read the TF info for cameras
  with open(osp.join(rospack.get_path(package_name),
    'calibrations/stereo.pkl'), 'r') as f:
    extrinsics = pickle.load(f)

  cam_trans = extrinsics['T']
  cam_rot = np.eye(4)
  cam_rot[0, :3] = extrinsics['R'][0, :]
  cam_rot[1, :3] = extrinsics['R'][1, :]
  cam_rot[2, :3] = extrinsics['R'][2, :]
  cam_q = tx.quaternion_from_matrix(cam_rot)
  cam_parent = 'kinect2_rgb_optical_frame'
  cam_child = 'boson_frame'

  # construct the static transform
  tform = TransformStamped()
  tform.header.stamp = rospy.Time.now()
  tform.header.frame_id = cam_parent
  tform.child_frame_id = cam_child
  tform.transform.translation.x = float(cam_trans[0])
  tform.transform.translation.y = float(cam_trans[1])
  tform.transform.translation.z = float(cam_trans[2])
  tform.transform.rotation.x = cam_q[0]
  tform.transform.rotation.y = cam_q[1]
  tform.transform.rotation.z = cam_q[2]
  tform.transform.rotation.w = cam_q[3]
  rospy.loginfo('TF broadcaster is set up')

  # send
  broadcaster = tf2_ros.StaticTransformBroadcaster()
  broadcaster.sendTransform(tform)
  rospy.spin()

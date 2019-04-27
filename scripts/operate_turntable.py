#!/usr/bin/python
"""
ROS node that operates the turntable in the CowTech Ciclop 3D scanner.
It sets the current position as 0 degrees, rotates the turntable through
360 degrees and publishes its pose continuously as it is rotating
"""
import logging
from horus.engine.driver import board
import rospy
from geometry_msgs.msg import TransformStamped
import argparse
from tf import transformations as tx
from tf import TransformListener
import tf2_ros
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
import sys
import cv2
import subprocess
import os
from PyQt4.QtGui import QApplication

from recording_gui import RecordingGUI

osp = os.path

class RosbagRecord:
  """
  from https://gist.github.com/marco-tranzatto/8be49b81b1ab371dcb5d4e350365398a
  """
  def __init__(self, record_folder, kinect_res, hand_pose=False):
    self.record_script = 'hand_pose_record.sh' if hand_pose else \
        'contactdb_record.sh'
    self.bag_filename_prefix = '-hand-pose' if hand_pose else ''
    self.node_name = 'hand_pose_recorder' if hand_pose else \
        'contactdb_recorder'
    if not osp.isfile(self.record_script):
      rospy.logerr('Script file {:s} does not exist'.
                   format(osp.join(os.getcwd(), self.record_script)))
      return
    self.record_folder = osp.expanduser(record_folder)
    if not osp.exists(self.record_folder):
      os.makedirs(self.record_folder)
      rospy.loginfo('Directory {:s} created'.format(self.record_folder))
    self.kinect_res = kinect_res

  def start_recording(self, object_name):
    # Start recording.
    command = "source {:s} {:s}/{:s}{:s} {:s} {:s}".format(self.record_script,
      self.record_folder, object_name, self.bag_filename_prefix, self.kinect_res,
      self.node_name)
    self.p = subprocess.Popen(command, stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              shell=True, cwd=os.getcwd(),
                              executable='/bin/bash')
    rospy.loginfo(rospy.get_name() + ' start recording.')

  def _terminate_ros_node(self):
    # Adapted from
    # http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    # https://answers.ros.org/question/275441/start-and-kill-rosbag-record-from-bash-shell-script/
    list_cmd = subprocess.Popen("rosnode list", shell=True,
                                stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for node in list_output.split("\n"):
      if node.startswith('/{:s}'.format(self.node_name)):
        os.system("rosnode kill " + node)

  def stop_recording_handler(self):
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    self._terminate_ros_node()
    if hasattr(self, 'p'):
      o = str(self.p.stdout.read())
      e = str(self.p.stderr.read())
      rospy.loginfo('recorder script stdout: {:s}\nstderr: {:s}'.
          format(o, e))


class TurnTableOperator(object):
  def __init__(self, serial_port, marker_id, contactdb_recorder, hand_pose_recorder,
      step=3, motor_speed=150, motor_acceleration=200, marker_stable_thresh=30):
    """"
    :param serial_port:
    :param marker_id:
    :param {contactdb,hand_pose}_recorder: Object of the RosbagRecord class
    :param step:
    :param motor_speed:
    :param motor_acceleration:
    :param marker_stable_thresh: no. of times marker needs to be seen
    continuously
    """
    self.step = step
    self.marker_id = marker_id
    self.contactdb_recorder = contactdb_recorder
    self.hand_pose_recorder = hand_pose_recorder
    self.tt_base_t = []
    self.tt_base_pose = TransformStamped()
    self.tf_listener = TransformListener()
    self.marker_seen_count = 0
    self.marker_stable_thresh = marker_stable_thresh
    self.marker_pose_ready = False
    self.tt_angle_bcaster = tf2_ros.TransformBroadcaster()
    self.tt_center_bcaster = tf2_ros.TransformBroadcaster()

    # logging for the Arduino driver
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    board.logger.addHandler(ch)

    # init and connect the driver
    self.arduino = board.Board()
    self.arduino.serial_name = serial_port
    try:
      self.arduino.connect()
    except Exception as e:
      rospy.logerr('Could not connect to turntable: {:s}'.format(e))
      raise e

    # init and configure the motor
    self.arduino.motor_invert(True)
    self.arduino.motor_enable()
    self.arduino.motor_reset_origin()
    self.arduino.motor_speed(motor_speed)
    self.arduino.motor_acceleration(motor_acceleration)

    # OpenCV window for clicking
    self.im_size = 50
    self.button_win_name = 'Marker Ready'
    self.button_im = self._make_im([0, 0, 255])
    cv2.imshow(self.button_win_name, self.button_im)
    cv2.waitKey(1)

    rospy.sleep(0.5)
    assert self.tf_listener.frameExists('turntable_base_tmp')
    assert self.tf_listener.frameExists('boson_frame')

  def _make_im(self, pixel):
    """
    :param pixel: (3, )
    :return:
    """
    im = np.asarray(pixel, dtype=np.uint8)[np.newaxis, np.newaxis, :]
    im = np.repeat(np.repeat(im, self.im_size, axis=0), self.im_size, axis=1)
    return im

  def publish_tfs(self, tt_angle):
    t = TransformStamped()  # pose of turntable_frame w.r.t. turntable_base
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'turntable_base'
    t.child_frame_id  = 'turntable_frame'
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    q = tx.quaternion_from_euler(0, 0, np.deg2rad(tt_angle))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    self.tt_angle_bcaster.sendTransform(t)

    t.header.frame_id = 'kinect2_rgb_optical_frame'
    t.child_frame_id = 'turntable_base'
    tm = np.mean(self.tt_base_t, axis=0)
    t.transform.translation.x = tm[0]
    t.transform.translation.y = tm[1]
    t.transform.translation.z = tm[2]
    t.transform.rotation = self.tt_base_pose.transform.rotation
    self.tt_center_bcaster.sendTransform(t)

  def record_hand_pose(self, object_name):
    self.hand_pose_recorder.start_recording(object_name)

  def run_turntable(self, object_name):
    # first stop the hand pose recorder node if it is active
    self.hand_pose_recorder.stop_recording_handler()
    # start recording for contactdb
    angle = 0
    self.contactdb_recorder.start_recording(object_name)
    rospy.sleep(1.5)
    while angle < 360:
      rospy.loginfo('Angle = {:d}'.format(angle))
      self.publish_tfs(tt_angle=angle)
      rospy.sleep(0.5)
      self.arduino.motor_move(self.step)
      angle += self.step
      rospy.sleep(0.5)
      cv2.waitKey(1)
    self.contactdb_recorder.stop_recording_handler()

  def disconnect(self):
    self.arduino.disconnect()
    rospy.loginfo('Disconnected from Arduino')
    self.contactdb_recorder.stop_recording_handler()
    self.hand_pose_recorder.stop_recording_handler()

  def ar_tag_callback(self, markers):
    cv2.imshow(self.button_win_name, self.button_im)
    cv2.waitKey(1)

    if self.marker_pose_ready:
      return

    for m in markers.markers:
      if m.id != self.marker_id:
        continue
      if self.marker_seen_count > self.marker_stable_thresh:
        self.button_im = self._make_im([0, 255, 0])
        self.marker_pose_ready = True
        self.marker_seen_count = self.marker_stable_thresh
      else:
        self.button_im = self._make_im([0, 0, 255])
        pos, rotquat = self.tf_listener.lookupTransform(
          'kinect2_rgb_optical_frame', 'turntable_base_tmp', rospy.Time(0))
        if np.isnan(pos).any() or np.isnan(rotquat).any():
          rospy.logwarn('NaN pose')
          break
        self.marker_seen_count += 1
        self.tt_base_t.append(pos)
        self.tt_base_pose.transform.rotation.x = rotquat[0]
        self.tt_base_pose.transform.rotation.y = rotquat[1]
        self.tt_base_pose.transform.rotation.z = rotquat[2]
        self.tt_base_pose.transform.rotation.w = rotquat[3]
        rospy.loginfo('Marker {:d} seen {:d} times'.
                      format(self.marker_id, self.marker_seen_count))
        rospy.loginfo('Current pose of TT base w.r.t. Kinect ='
                      '[{:4.3f}, {:4.3f}, {:4.3f}]'.
                      format(self.tt_base_pose.transform.translation.x,
                             self.tt_base_pose.transform.translation.y,
                             self.tt_base_pose.transform.translation.z))
      break
    else:
      rospy.logwarn('Marker {:d} not seen, resetting counter'.
                    format(self.marker_id))
      # self.marker_seen_count = 0

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--serial_port', type=str, default='/dev/ttyACM0',
                      help='Serial port of the Arduino')
  parser.add_argument('--marker_id', type=int, default=0,
                      help='ID of the marker')
  parser.add_argument('--data_dir', type=str,
                      default=osp.join('..', 'data', 'contactdb_data'))
  parser.add_argument('--kinect_res', type=str, default='qhd')

  rospy.init_node('turntable_operator')
  myargv = rospy.myargv(argv=sys.argv)
  args = parser.parse_args(myargv[1:])

  contactdb_recorder = RosbagRecord(args.data_dir, args.kinect_res,
      hand_pose=False)
  hand_pose_recorder = RosbagRecord(args.data_dir, args.kinect_res,
      hand_pose=True)
  tt = TurnTableOperator(contactdb_recorder=contactdb_recorder,
      hand_pose_recorder=hand_pose_recorder, serial_port=args.serial_port,
      marker_id=int(args.marker_id), step=40)
  rospy.on_shutdown(tt.disconnect)
  rospy.Subscriber('deepgrasp/ar_pose_marker', AlvarMarkers,
                   tt.ar_tag_callback)

  def record_cb(object_name, hand_pose=False):
    if not tt.marker_pose_ready:
      rospy.logwarn('Marker pose not captured, not recording')
      return

    if hand_pose:
      tt.record_hand_pose(object_name)
    else:
      if tt.arduino._is_connected:
        tt.run_turntable(object_name)
      else:
        rospy.logwarn('Turntable is not connected, not recording')
  def contactdb_cb(object_name):
    record_cb(object_name=object_name, hand_pose=False)
  def hand_pose_cb(object_name):
    record_cb(object_name=object_name, hand_pose=True)

  # read list of object
  with open('object_names.txt', 'r') as f:
    object_list = [l.rstrip() for l in f]
  object_list = sorted(object_list)

  # Qt stuff
  app = QApplication(sys.argv)
  gui = RecordingGUI(object_list,
      contactdb_recording_cb=contactdb_cb,
      hand_pose_recording_cb=hand_pose_cb)
  gui.show()
  sys.exit(app.exec_())

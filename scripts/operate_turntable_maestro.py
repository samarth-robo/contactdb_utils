"""
ROS node that
(1) Resets the turntable to 0 degrees
(2) Waits for pose estimation to finish
(3) Records bag file of the following while rotating the turntable:
  - RGB image
  - Thermal image
  - Depth image
  - Turntable angle
"""
from Maestro.maestro import Controller as Turntable
import time

class TurntableRecorder(object):
  def __init__(self, chan=5, min_pw=480, max_pw=2256, speed=12):
    """
    :param min_pw: pulse width for 0 degrees (ms)
    :param max_pw: pulse width for 180 degrees (ms)
    :param chan: channel of servo connection on the Maestro board
    """
    self.tt = Turntable()
    self.chan = chan
    self.tt.setRange(self.chan, min=min_pw*4, max=max_pw*4)
    self.tt.setSpeed(self.chan, speed=speed)

  def _deg2cmd(self, deg):
    """
    Converts a degree value to the pulse width in quarter milliseconds
    :param deg:
    :return: command to be sent to the servo
    """
    min_o = self.tt.getMin(self.chan)
    max_o = self.tt.getMax(self.chan)
    m = (max_o - min_o) / 180.0
    c = min_o
    out = int(m * deg + c)
    return out

  def set_yaw(self, deg):
    """
    Commands the turntable to go to a yaw angle
    :param deg: angle in degrees
    :return: blocks until the turntable is moving
    """
    self.tt.setTarget(self.chan, self._deg2cmd(deg))
    while self.tt.isMoving(self.chan):
      pass

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.tt.close()

if __name__ == '__main__':
  with TurntableRecorder() as tr:
    print('0 degrees...')
    tr.set_yaw(0)
    print('Done')
    time.sleep(2)
    print('110 degrees...')
    tr.set_yaw(110)
    print('Done')
    time.sleep(2)
    print('170 degrees...')
    tr.set_yaw(170)
    print('Done')

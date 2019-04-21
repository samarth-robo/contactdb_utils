import sys
from PyQt4.QtCore import pyqtSlot
from PyQt4.QtGui import *
from functools import partial
import time

class RecordingGUI(QWidget):
  def __init__(self, names, contactdb_recording_cb, hand_pose_recording_cb,
      *args, **kwargs):
    super(RecordingGUI, self).__init__(*args, **kwargs)
    ncols = 5
    self.contactdb_recording_cb = contactdb_recording_cb
    self.hand_pose_recording_cb = hand_pose_recording_cb

    self.setWindowTitle('Select objects')

    # grid of radio buttons
    self.object_grid = QGridLayout()
    self.buttons = {name: QRadioButton(name, parent=self) for name in names}
    for idx, name in enumerate(names):
      self.buttons[name].clicked.connect(partial(self.on_click, name=name))
      c = idx % ncols
      r = idx / ncols
      self.object_grid.addWidget(self.buttons[name], r, c)
    self.buttons[names[0]].setChecked(True)
    self.current_object = names[0]

    # record buttons
    self.button_grid = QGridLayout()
    self.hand_pose_record_button = QPushButton('Record Hand Pose',
        parent=self)
    self.hand_pose_record_button.clicked.connect(partial(self.record_cb,
      hand_pose=True))
    self.button_grid.addWidget(self.hand_pose_record_button, 0, 0)
    self.contactdb_record_button = QPushButton('Record ContactDB',
        parent=self)
    self.contactdb_record_button.clicked.connect(partial(self.record_cb,
      hand_pose=False))
    self.button_grid.addWidget(self.contactdb_record_button, 0, 1)

    # main layout
    self.vlayout = QVBoxLayout()
    self.vlayout.addLayout(self.object_grid)
    self.vlayout.addLayout(self.button_grid)
    self.setLayout(self.vlayout)

  @pyqtSlot()
  def on_click(self, name):
    self.current_object = name
    print('Current object = {:s}'.format(name))

  @pyqtSlot()
  def record_cb(self, hand_pose):
    if hand_pose:
      print('Recording {:s} hand pose...'.format(self.current_object))
      self.hand_pose_recording_cb(object_name=self.current_object)
    else:
      print('Recording {:s} ContactDB...'.format(self.current_object))
      self.contactdb_recording_cb(object_name=self.current_object)
    print('Done')


if __name__ == '__main__':
  # create our window
  app = QApplication(sys.argv)

  # Show the window and run the app
  def recording_fn(object_name):
    time.sleep(1)
  gui = RecordingGUI(['a', 'b', 'c', 'd'], recording_fn, recording_fn)
  gui.show()
  sys.exit(app.exec_())

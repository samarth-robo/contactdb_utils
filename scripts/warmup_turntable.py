from horus.engine.driver import board
import time

# init and connect the driver
arduino = board.Board()
arduino.serial_name = '/dev/ttyACM0'
try:
  arduino.connect()
except Exception as e:
  print 'Could not connect to turntable: {:s}'.format(e)
  raise e


arduino.motor_invert(True)
arduino.motor_enable()
arduino.motor_reset_origin()
arduino.motor_speed(200)
arduino.motor_acceleration(200)

step = 40
N = 50

for i in xrange(N):
  if i % 5 == 0:
    print 'Step {:d} / {:d}'.format(i, N)
  arduino.motor_move(step=step)
  time.sleep(1)

arduino.disconnect()

import struct
import time
import serial

from geometry_msgs.msg import Pose2D


class Serializer(object):
   def __init__(self):
      self.sampleRate = 50 #samples per second
      self.pulsePerRotation = 4955 #Old motors
      self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux
   def set_speed(self, vw1, vw2, vw3):
      self.setSpeed(vw1*self.pulsePerRotation, vw2*self.pulsePerRotation,
                    vw3*self.pulsePerRotation)
   # Serial Communication Functions #
   def writeFloat(self, f):
      self.ser.write(struct.pack('>i', int(f*1000)))
   def readFloat(self):
      return float(struct.unpack('>i', self.ser.read(4))[0])/1000
   def setPower(self, p1, p2, p3):
      self.ser.write('p')
      self.writeFloat(p1)
      self.writeFloat(p2)
      self.writeFloat(p3)
   def setSpeed(self, s1, s2, s3):
      self.ser.write('s')
      self.writeFloat(s1)
      self.writeFloat(s2)
      self.writeFloat(s3)
   def setPID(self, motor, p, i, qpps): #use motor = 0 to set all motors
      self.ser.write('k')
      self.ser.write(str(motor))
      self.writeFloat(p)
      self.writeFloat(i)
      self.writeFloat(qpps)
   def setT(self, period_ms, tau_ms):
      self.ser.write('t')
      self.writeFloat(period_ms)
      self.writeFloat(tau_ms)
   def getSpeed(self):
      self.ser.write('v')
      return readFloat(), readFloat(), readFloat()
   def getEncoderCount(self):
      self.ser.write('e')
      return readFloat(), readFloat(), readFloat()
   def disengage(self):
      self.ser.write('d')

# Run #
def main():
   ser = Serializer()
   #ser = serial.Serial('COM11', 115200, timeout=None) #windows

   # Set the PIDQ values for all motors
   #setPID(0, 1, 1, 800)
   ser.setPID(0, 1.5, 3, 100000)

   # Set tick period (triggers PID control) and velocity filter corner frequency
   ser.setT(20, 50)

   #setPower(80, 80, 80)
   #setSpeed(0, 0, 0)
   ser.setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation,
                speedM3*pulsePerRotation)
   time.sleep(3)
   ser.disengage()


if __name__ == '__main__':
   main()

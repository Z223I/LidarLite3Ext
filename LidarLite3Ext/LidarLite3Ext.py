import sys
sys.path.append("/home/pi/pythondev/Lidar-Lite/python")
# This is the threaded version.


import time
from threading   import Thread
from lidar_lite  import Lidar_Lite
from collections import deque
import queue
import pdb



class LidarLite3Ext(Lidar_Lite):
  'LidarLite3Ext documentation'

  def __init__(self):
    super( LidarLite3Ext, self ).__init__()
#    print ("LidarLite3Ext constructor")
    self._running = True
    self.simulatedData = False

  def init(self):

#    pdb.set_trace()
    connected = self.connect(1)

#    print("Connected = ", connected)

    if connected >= 0:
#      print ("Lidar connected")

      try:
        self.writeAndWait( 0x04, 0x0A )
        self.writeAndWait( 0x11, 0x0A ) # Distance measurements per request.  Using 10.
        self.writeAndWait( 0x1C, 0x60 ) # Reduce sensitivity and errors per manual.
      except:
        print ("Lidar not available.")
        print ("Using simulated data.")
        self.simulatedData = True

      return True
#end if

    else:
      print ("Lidar not connected.")
      return False

  def read(self):
        # Read current range.
        distance_cm = self.getDistance()
        distance_inch = distance_cm / 2.54
        return distance_inch


  def terminate(self):
    self._running = False
  
  def run(self, _qDistance):
    xRange = []
    maxItemsInQueue = 1
    measurements = deque(xRange, maxItemsInQueue)

    while self._running:

      if self.simulatedData:
        distanceCM = 25.4
        distanceInch = distanceCM / 2.54
      else:
        distanceCM = self.getDistance()
        distanceInch = distanceCM / 2.54

#      print "Inches:  ", distanceInch

      # Currently the averaging block of code isn't being used.
      averaging = False

      if averaging:
        measurements.appendleft( distanceInch )
        sumOfMeasurements = sum( measurements )
        average = sumOfMeasurements / len( measurements )
        #print 'Running average Inches: {:.2f}'.format(average)
        _qDistance.put(average)
      else:
        _qDistance.put( distanceInch )
#        print ( "Laser detector distance: ", distanceInch )


#    velocityMetersPerSecond = lidar.getVelocity()
#    velocityInchesPerSecond = velocityMetersPerSecond / 39.3700787
#    velocityInchesPerMinute = velocityInchesPerSecond * 60
 
    #print "Inches per minute: ", velocityInchesPerMinute 

      time.sleep(2.5)

  def write_ext(self, register, value, lidarliteAddress):
      self.address = lidarliteAddress
      self.writeAndWait(register, value)


  """------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
  ------------------------------------------------------------------------------"""

  def configure(self, configuration, lidarliteAddress):

    if configuration == 0:
      # Default mode, balanced performance
      self.write_ext(0x02,0x80,lidarliteAddress); # Default
      self.write_ext(0x04,0x08,lidarliteAddress); # Default
      self.write_ext(0x1c,0x00,lidarliteAddress); # Default
    elif configuration == 1:
      # Short range, high speed
      self.write_ext(0x02,0x1d,lidarliteAddress);
      self.write_ext(0x04,0x08,lidarliteAddress); # Default
      self.write_ext(0x1c,0x00,lidarliteAddress); # Default

    elif configuration == 2:
      # Default range, higher speed short range
      self.write_ext(0x02,0x80,lidarliteAddress); # Default
      self.write_ext(0x04,0x00,lidarliteAddress);
      self.write_ext(0x1c,0x00,lidarliteAddress); # Default

    elif configuration == 3:
      # Maximum range
      self.write_ext(0x02,0xff,lidarliteAddress);
      self.write_ext(0x04,0x08,lidarliteAddress); # Default
      self.write_ext(0x1c,0x00,lidarliteAddress); # Default

    elif configuration ==4:
      # High sensitivity detection, high erroneous measurements
      self.write_ext(0x02,0x80,lidarliteAddress); # Default
      self.write_ext(0x04,0x08,lidarliteAddress); # Default
      self.write_ext(0x1c,0x80,lidarliteAddress);

    elif configuration == 5:
      # Low sensitivity detection, low erroneous measurements
      self.write_ext(0x02,0x80,lidarliteAddress); # Default
      self.write_ext(0x04,0x08,lidarliteAddress); # Default
      self.write_ext(0x1c,0xb0,lidarliteAddress);

# End configure


if __name__ == "__main__":

  #Create Class
  lidarLiteExt = LidarLite3Ext()

  initOk = lidarLiteExt.init()

  if initOk:

    # run through all the configurations
    for config in range(6):
        lidarLiteExt.configure(config, 0x62)

    qDistance = queue.Queue(maxsize=0)

    #Create Thread
    lidarLiteExtThread = Thread(target=lidarLiteExt.run, args=(qDistance,))

    #Start Thread
    lidarLiteExtThread.start()

    while True:
      distance = qDistance.get()
      print ("Distance: ", distance)

    lidarLiteExt.terminate()
    print ("Thread finished")

  else:
    print ("Shutting down")


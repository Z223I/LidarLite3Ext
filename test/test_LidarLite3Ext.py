try:
    from builtins import object
except ImportError:
    pass

#import warnings

#from unittest import TestCase  #, skipIf
import unittest
import time

import sys
sys.path.append('/home/pi/pythondev/LidarLite3Ext/LidarLite3Ext')
from LidarLite3Ext import LidarLite3Ext

class TestLidarLite3Ext(unittest.TestCase):

    def setUp(self):
        self.test_lidar_lite_3_ext = LidarLite3Ext()

    def tearDown(self):
        pass

    def test___init___(self):

    def test_(self):
        pass




if __name__ == "__main__":

    unittest.main()

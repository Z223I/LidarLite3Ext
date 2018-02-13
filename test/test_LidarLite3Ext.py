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
        self.test_lidar_lite_3_ext.init()

    def tearDown(self):
        pass

    def test___init___(self):
        pass

    def test_auto_configure_A(self):
        """test_auto_configure_A tests SHORT_RANGE."""
        INCHES_PER_FOOT = 12
        distance_inches = 24 * INCHES_PER_FOOT
        configuration = self.test_lidar_lite_3_ext.auto_configure(distance_inches)
        self.assertEqual(LidarLite3Ext.SHORT_RANGE, configuration)

    def test_auto_configure_B(self):
        """test_auto_configure_B tests DEFAULT_RANGE."""
        INCHES_PER_FOOT = 12
        distance_inches = 26 * INCHES_PER_FOOT
        configuration = self.test_lidar_lite_3_ext.auto_configure(distance_inches)
        self.assertEqual(LidarLite3Ext.DEFAULT_RANGE, configuration)
        distance_inches = 79 * INCHES_PER_FOOT
        configuration = self.test_lidar_lite_3_ext.auto_configure(distance_inches)
        self.assertEqual(LidarLite3Ext.DEFAULT_RANGE, configuration)

    def test_auto_configure_C(self):
        """test_auto_configure_C tests SHORT_RANGE."""
        INCHES_PER_FOOT = 12
        distance_inches = 81 * INCHES_PER_FOOT
        configuration = self.test_lidar_lite_3_ext.auto_configure(distance_inches)
        self.assertEqual(LidarLite3Ext.MAXIMUM_RANGE, configuration)




if __name__ == "__main__":

    unittest.main()

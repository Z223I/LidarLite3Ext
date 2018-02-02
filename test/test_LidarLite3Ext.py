try:
    from builtins import object
except ImportError:
    pass

#import warnings

#from unittest import TestCase  #, skipIf
import unittest
import time

import sys
sys.path.append('/home/pi/pythondev/RangeBot/RangeBot')
from RangeBot import RangeBot

class TestRangeBot(unittest.TestCase):
    CHANNEL = 4

    def setUp(self):
        self.test_range_bot = RangeBot(TestRangeBot.CHANNEL)

    def tearDown(self):
        pass

    def test___init___(self):
        self.assertEqual(self.test_range_bot.clip_distance, 10)

    def test_set_clip_distance(self):
        val = 6
        self.test_range_bot.set_clip_distance(val)
        self.assertEqual(self.test_range_bot.clip_distance, val)

    def test_scan(self):
        pass

    def test_find_target2(self):
        pass

    def test_execute(self):
        pass

    def test_valid_hunt_A(self):
        hits = 7
        ranges = [100,100,100,100,100,100,100, \
            24, 24, 24, 24, 24, 24, 24, \
            100,100,100,100,100,100,100]

        valid = self.test_range_bot.valid_hunt(ranges, hits)
        self.assertTrue(valid)

    def test_valid_hunt_B(self):
        # Four hits is below the threshold for a valid hunt.
        hits = 4

        # The values of ranges is irrelavent.
        ranges = [100,100,100,100,100,100,100, \
            24, 24, 24, 24, 24, 24, 24, \
            100,100,100,100,100,100,100]

        valid = self.test_range_bot.valid_hunt(ranges, hits)
        self.assertFalse(valid)

    def test_execute_hunt(self):
        pass




if __name__ == "__main__":

    unittest.main()

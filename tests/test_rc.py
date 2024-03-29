import unittest

import pymavclient


class TestRC(unittest.TestCase):
    def setUp(self):
        self.simulation = pymavclient.PyMAVSim()
        self.simulation.start()
        self.vehicle = pymavclient.PyMAVClient()

    def tearDown(self):
        self.vehicle.close()
        self.simulation.close()

    def test_rc(self):
        self.assertEqual(first=self.vehicle.get_rc_channel(1), second=1500,
                         msg="RC channel 1 (roll) is not 1000")
        self.assertEqual(first=self.vehicle.get_rc_channel(2), second=1500,
                         msg="RC channel 2 (pitch) is not 1000")
        self.assertEqual(first=self.vehicle.get_rc_channel(3), second=1000,
                         msg="RC channel 3 (throttle) is not 1000")
        self.assertEqual(first=self.vehicle.get_rc_channel(4), second=1500,
                         msg="RC channel 4 (yaw) is not 1000")

        self.vehicle.set_rc_channel(channel=6, value=2000)
        self.assertTrue(expr=self.vehicle.wait_rc_channel(channel=6, value=2000, timeout=10),
                        msg="RC channel 6 is not set to 2000")

        self.vehicle.set_rc_channel(channel=6, value=1000)
        self.assertTrue(expr=self.vehicle.wait_rc_channel(channel=6, value=1000, timeout=10),
                        msg="RC channel 6 is not set to 1000")

        self.vehicle.set_rc_channels(channels_and_values={6: 2000, 7: 1800, 8: 1200})
        self.assertTrue(expr=self.vehicle.wait_rc_channel(channel=6, value=2000, timeout=10),
                        msg="RC channel 6 is not set to 2000")
        self.assertTrue(expr=self.vehicle.wait_rc_channel(channel=7, value=1800, timeout=10),
                        msg="RC channel 7 is not set to 1800")
        self.assertTrue(expr=self.vehicle.wait_rc_channel(channel=8, value=1200, timeout=10),
                        msg="RC channel 8 is not set to 1200")

import time
import unittest
import pymavclient


class TestMode(unittest.TestCase):
    def setUp(self):
        self.simulation = pymavclient.PyMAVSim()
        self.simulation.start()
        self.vehicle = pymavclient.PyMAVClient()

    def tearDown(self):
        self.vehicle.close()
        self.simulation.close()

    def test_mode(self):
        self.vehicle.change_mode(mode="GUIDED")
        time.sleep(2)
        self.assertEqual(first=self.vehicle.mode, second="GUIDED", msg="Vehicle failed to change mode to GUIDED")

        self.vehicle.change_mode(mode="STABILIZE")
        time.sleep(2)
        self.assertEqual(first=self.vehicle.mode, second="STABILIZE", msg="Vehicle failed to change mode to STABILIZE")

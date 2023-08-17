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
        self.assertTrue(expr=self.vehicle.wait_mode(mode="GUIDED"), msg="Vehicle failed to change mode to GUIDED")
        self.vehicle.change_mode(mode="STABILIZE")
        self.assertTrue(expr=self.vehicle.wait_mode(mode="STABILIZE"), msg="Vehicle failed to change mode to STABILIZE")
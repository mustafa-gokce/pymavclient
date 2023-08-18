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
        self.assertTrue(expr=self.vehicle.wait_mode(mode="STABILIZE", timeout=10),
                        msg="Vehicle mode is not to STABILIZE")

        self.vehicle.change_mode(mode="GUIDED")
        self.assertTrue(expr=self.vehicle.wait_mode(mode="GUIDED", timeout=10),
                        msg="Vehicle failed to change mode to GUIDED")

        self.vehicle.change_mode(mode="STABILIZE")
        self.assertTrue(expr=self.vehicle.wait_mode(mode="STABILIZE", timeout=10),
                        msg="Vehicle failed to change mode to STABILIZE")

import unittest

import pymavclient


class TestArm(unittest.TestCase):
    def setUp(self):
        self.simulation = pymavclient.PyMAVSim()
        self.simulation.start()
        self.vehicle = pymavclient.PyMAVClient()

    def tearDown(self):
        self.vehicle.close()
        self.simulation.close()

    def test_arm(self):
        self.assertTrue(expr=self.vehicle.wait_disarmed(), msg="Vehicle is not disarmed")

        self.vehicle.arm()
        self.assertTrue(expr=self.vehicle.wait_armed(), msg="Vehicle is failed to arm")

        self.vehicle.disarm()
        self.assertTrue(expr=self.vehicle.wait_disarmed(), msg="Vehicle is failed to disarm")

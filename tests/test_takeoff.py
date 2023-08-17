import unittest

import pymavclient


class TestTakeoff(unittest.TestCase):
    def setUp(self):
        self.simulation = pymavclient.PyMAVSim()
        self.simulation.start()
        self.vehicle = pymavclient.PyMAVClient()

    def tearDown(self):
        self.vehicle.close()
        self.simulation.close()

    def test_takeoff(self):
        self.vehicle.change_mode(mode="GUIDED")
        self.assertTrue(expr=self.vehicle.wait_mode(mode="GUIDED"), msg="Vehicle failed to change mode to GUIDED")

        self.assertTrue(expr=self.vehicle.wait_armable(wait_before=5), msg="Vehicle is failed to be armable")

        self.vehicle.arm()
        self.assertTrue(expr=self.vehicle.wait_armed(), msg="Vehicle is failed to arm")

        self.vehicle.takeoff(altitude=30)
        self.assertTrue(expr=self.vehicle.wait_altitude(altitude=30, timeout=30), msg="Vehicle failed to takeoff")

        self.vehicle.land()
        self.assertTrue(expr=self.vehicle.wait_disarmed(timeout=45), msg="Vehicle failed to land")

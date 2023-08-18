import unittest

import pymavclient


class TestFly(unittest.TestCase):
    def setUp(self):
        self.simulation = pymavclient.PyMAVSim()
        self.simulation.start()
        self.vehicle = pymavclient.PyMAVClient()

    def tearDown(self):
        self.vehicle.close()
        self.simulation.close()

    def test_fly(self):
        self.vehicle.change_mode(mode="GUIDED")
        self.assertTrue(expr=self.vehicle.wait_mode(mode="GUIDED", timeout=10),
                        msg="Vehicle failed to change mode to GUIDED")

        self.assertTrue(expr=self.vehicle.wait_armable(timeout=30, wait_before=5),
                        msg="Vehicle is failed to be armable")

        self.vehicle.arm()
        self.assertTrue(expr=self.vehicle.wait_armed(timeout=10),
                        msg="Vehicle is failed to arm")

        self.vehicle.takeoff(altitude=30)
        self.assertTrue(expr=self.vehicle.wait_altitude(altitude=30, timeout=30, absolute=False, precision=1.0),
                        msg="Vehicle failed to takeoff")

        self.vehicle.fly_to(latitude=-35.36130812, longitude=149.16114736, altitude=30, absolute=False)
        self.assertTrue(expr=self.vehicle.wait_location_3d(latitude=-35.36130812, longitude=149.16114736, altitude=30,
                                                           absolute=False, timeout=60, precision=1.0),
                        msg="Vehicle failed to fly to target position")

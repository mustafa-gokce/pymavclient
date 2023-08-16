import time
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

    def test_init(self):
        self.assertTrue(expr=self.vehicle.disarmed, msg="Vehicle is not disarmed")

    def test_arm(self):
        self.vehicle.arm()
        time.sleep(2)
        self.assertTrue(expr=self.vehicle.armed, msg="Vehicle is failed to arm")

    def test_disarm(self):
        self.vehicle.arm()
        time.sleep(2)

        self.vehicle.disarm()
        time.sleep(2)
        self.assertTrue(expr=self.vehicle.disarmed, msg="Vehicle is failed to disarm")

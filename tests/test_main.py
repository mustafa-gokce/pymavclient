import unittest
import pymavclient


class TestMain(unittest.TestCase):
    def setUp(self):
        self.simulation = pymavclient.PyMAVSim()
        self.simulation.start()
        self.vehicle = pymavclient.PyMAVClient(wait_connected=False, wait_ready=False,
                                               wait_parameters=False, wait_armable=False)

    def tearDown(self):
        self.vehicle.close()
        self.simulation.close()

    def test_vehicle_wait_connected(self):
        self.assertTrue(expr=self.vehicle.wait_connected(), msg="Timed out waiting for connection")

    def test_vehicle_wait_parameters(self):
        self.assertTrue(expr=self.vehicle.wait_parameters(), msg="Timed out waiting for parameters")

    def test_vehicle_wait_ready(self):
        self.assertTrue(expr=self.vehicle.wait_ready(), msg="Timed out waiting for ready")

    def test_vehicle_wait_armable(self):
        self.assertTrue(expr=self.vehicle.wait_armable(), msg="Timed out waiting for vehicle to be armable")

    def test_vehicle_mode(self):
        self.assertEqual(first=self.vehicle.mode, second="STABILIZE", msg="Vehicle mode is not STABILIZE")

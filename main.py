import time
import pymavclient

simulation = pymavclient.PyMAVSim()

print("Starting simulation")
simulation.start()
print("Simulation started")

vehicle = pymavclient.PyMAVClient(connection_string="tcp:127.0.0.1:5760",
                                  auto_connect=False, request_default_streams=False,
                                  wait_connected=False, wait_ready=False, wait_parameters=False, wait_armable=False)

print("Connecting to vehicle")
vehicle.connect()

if not vehicle.wait_connected():
    raise Exception("Timed out waiting for connection")
print("Connected to vehicle")

vehicle.request_default_streams()

if not vehicle.wait_parameters():
    raise Exception("Timed out waiting for parameters")
print("Got vehicle parameters")

if not vehicle.wait_ready():
    raise Exception("Timed out waiting for ready")
print("Vehicle is now ready")

if not vehicle.wait_armable():
    raise Exception("Timed out waiting for vehicle to be armable")
print("Vehicle can be armed now")

while True:
    try:
        print(f"Mode: {vehicle.mode}, "
              f"Armable: {vehicle.armable}, "
              f"Armed: {vehicle.armed}, "
              f"Latitude: {vehicle.latitude}, "
              f"Longitude: {vehicle.longitude}, "
              f"Altitude: {vehicle.absolute_altitude}")
        time.sleep(1)
    except KeyboardInterrupt:
        print("Closing vehicle connection")
        vehicle.close()
        print("Vehicle connection closed")
        print("Closing simulation")
        simulation.close()
        print("Simulation closed")
        break

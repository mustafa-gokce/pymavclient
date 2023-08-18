import threading
import time
import math
import datetime

import geopy.distance
import pymavlink.dialects.v20.all as dialect
import pymavlink.mavutil as utility


class PyMAVClient:
    """
    Just another Python client for MAVLink enabled vehicles
    """

    def __init__(self, connection_string="tcp:127.0.0.1:5760",
                 auto_connect=True,
                 baud_rate=115200, request_default_streams=True,
                 wait_connected=True, wait_connected_timeout=30,
                 wait_ready=True, wait_ready_timeout=60,
                 wait_parameters=True, wait_parameters_timeout=30,
                 auto_reconnect=True, auto_reconnect_timeout=5,
                 wait_armable=True, wait_armable_timeout=30):
        """
        Initialize PyMAVClient.
        """

        self.__vehicle = None
        self.__connected = False
        self.__ready = False
        self.__last_message_time = -1
        self.__auto_connect = auto_connect
        self.__reconnect = auto_reconnect
        self.__reconnect_timeout = auto_reconnect_timeout
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.__wait_connected_timeout = wait_connected_timeout
        self.__wait_ready_timeout = wait_ready_timeout
        self.__wait_parameters_timeout = wait_parameters_timeout
        self.__wait_armable_timeout = wait_armable_timeout
        self.__messages = {}
        self.__parameters = {}
        self.__flight_modes = {}
        self.__should_be_connected = True
        self.__vehicle_connection_thread = None
        if self.__auto_connect:
            self.connect()
        if wait_connected:
            self.wait_connected(timeout=self.__wait_connected_timeout)
        if request_default_streams:
            self.request_default_streams()
        if wait_ready:
            self.wait_ready(timeout=self.__wait_ready_timeout)
        if wait_parameters:
            self.wait_parameters(timeout=self.__wait_parameters_timeout)
        if wait_armable:
            self.wait_armable(timeout=self.__wait_armable_timeout)

    def __vehicle_connection(self):
        """
        Connect to the vehicle.
        """
        while self.__should_be_connected:
            try:
                self.__vehicle = utility.mavlink_connection(device=self.connection_string,
                                                            baud=self.baud_rate,
                                                            autoreconnect=False,
                                                            retries=0,
                                                            force_connected=False)
                assert self.__vehicle.wait_heartbeat(blocking=True, timeout=self.__reconnect_timeout) is not None
                self.__last_message_time = time.monotonic()
                self.__connected = True
                self.__flight_modes = self.__vehicle.mode_mapping()
                while self.__should_be_connected:
                    try:
                        if self.__reconnect and time.monotonic() - self.__last_message_time > self.__reconnect_timeout:
                            break
                        message = self.__vehicle.recv_msg()
                        if message is None:
                            time.sleep(0.01)
                            continue
                        self.__last_message_time = time.monotonic()
                        message_dict = message.to_dict()
                        message_dict["monotonic"] = self.__last_message_time
                        self.__messages[message.msgname] = message_dict
                        if message.msgname == "PARAM_VALUE":
                            self.__parameters[message.param_id] = message_dict
                    except Exception as e:
                        time.sleep(0.01)
                self.__connected = False
                self.__vehicle.close()
            except Exception as e:
                self.__connected = False
                if self.__vehicle is not None:
                    self.__vehicle.close()
                time.sleep(0.01)

    def connect(self):
        """
        Connect to the vehicle.
        """
        self.__should_be_connected = True
        if self.__vehicle_connection_thread is None:
            self.__vehicle_connection_thread = threading.Thread(target=self.__vehicle_connection)
            self.__vehicle_connection_thread.start()

    def close(self):
        """
        Close the vehicle connection.
        """
        self.__should_be_connected = False
        if self.__vehicle_connection_thread is not None:
            self.__vehicle_connection_thread.join(timeout=self.__wait_connected_timeout)
        self.__vehicle_connection_thread = None

    def wait_connected(self, timeout=30):
        """
        Wait for the vehicle to connect.
        """
        start_time = time.monotonic()
        while not self.__connected:
            if time.monotonic() - start_time > timeout > 0:
                return False
            time.sleep(0.1)
        return True

    def request_default_streams(self, rate=4):
        """
        Request default streams from the vehicle.
        """
        if self.wait_connected():
            message = dialect.MAVLink_request_data_stream_message(target_system=self.__vehicle.target_system,
                                                                  target_component=self.__vehicle.target_component,
                                                                  req_stream_id=0,
                                                                  req_message_rate=rate,
                                                                  start_stop=1)
            self.__vehicle.mav.send(message)

    def cancel_default_streams(self):
        """
        Cancel default streams from the vehicle.
        """
        if self.wait_connected():
            message = dialect.MAVLink_request_data_stream_message(target_system=self.__vehicle.target_system,
                                                                  target_component=self.__vehicle.target_component,
                                                                  req_stream_id=0,
                                                                  req_message_rate=0,
                                                                  start_stop=0)
            self.__vehicle.mav.send(message)

    def wait_ready(self, timeout=60):
        """
        Wait for the vehicle to be ready.
        """
        self.wait_connected()
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout:
                self.__ready = False
                return False
            if "ATTITUDE" in self.__messages and "GLOBAL_POSITION_INT" in self.__messages:
                break
            time.sleep(0.1)
        if "HOME_POSITION" not in self.__messages:
            self.request_message_stream(message=dialect.MAVLINK_MSG_ID_HOME_POSITION, rate=1)
        while True:
            if time.monotonic() - start_time > timeout:
                self.__ready = False
                return False
            if "HOME_POSITION" in self.__messages:
                self.cancel_message_stream(message=dialect.MAVLINK_MSG_ID_HOME_POSITION)
                self.__ready = True
                return True
            time.sleep(0.1)

    def __wait_parameters(self, timeout=10, request_before=True, request_after=True):
        """
        Wait for the vehicle to receive parameters.
        """
        start_time = time.monotonic()
        self.wait_connected()
        if request_before:
            self.send_request_parameter_list()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                break
            if len(self.__parameters) > 0:
                local_parameter_list_length = len(self.__parameters)
                remote_parameter_list_length = self.__messages.get("PARAM_VALUE", {}).get("param_count", 0)
                if local_parameter_list_length >= remote_parameter_list_length:
                    return True
            time.sleep(0.1)
        if request_after:
            remote_parameter_list_length = self.__messages.get("PARAM_VALUE", {}).get("param_count", 0)
            local_parameter_indexes = [parameter["param_index"] for parameter in list(self.__parameters.values())]
            for parameter_index in range(remote_parameter_list_length):
                if parameter_index not in local_parameter_indexes:
                    self.send_request_parameter(parameter_index=parameter_index)
        return False

    def wait_parameters(self, timeout=30, request_before=True, request_after=True):
        """
        Wait for the vehicle to receive parameters.
        """
        start_time = time.monotonic()
        __timeout = int(timeout / 3)
        self.wait_connected()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.__wait_parameters(timeout=__timeout, request_before=request_before, request_after=request_after):
                return True

    def wait_armable(self, timeout=30, steady_time=5):
        """
        Wait for the vehicle to be able to arm.
        """
        if not timeout > steady_time > 0:
            return False
        end_time = time.monotonic() + timeout - steady_time
        if not self.wait_connected(timeout=timeout):
            return False
        while True:
            if time.monotonic() > end_time:
                return False
            if self.armable:
                time.sleep(steady_time)
                if self.armable:
                    return True
            time.sleep(0.1)

    def send_request_parameter_list(self):
        """
        Send a request parameter list to the vehicle.
        """
        if self.wait_connected():
            message = dialect.MAVLink_param_request_list_message(target_system=self.__vehicle.target_system,
                                                                 target_component=self.__vehicle.target_component)
            self.__vehicle.mav.send(message)

    def send_request_parameter(self, parameter_name="", parameter_index=-1):
        """
        Send a request parameter to the vehicle.
        """
        if self.wait_connected():
            message = dialect.MAVLink_param_request_read_message(target_system=self.__vehicle.target_system,
                                                                 target_component=self.__vehicle.target_component,
                                                                 param_id=parameter_name.encode("utf-8"),
                                                                 param_index=parameter_index)
            self.__vehicle.mav.send(message)

    def send_command_long(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """
        Send a command to the vehicle.
        """
        if self.wait_connected():
            message = dialect.MAVLink_command_long_message(target_system=self.__vehicle.target_system,
                                                           target_component=self.__vehicle.target_component,
                                                           command=command,
                                                           confirmation=0,
                                                           param1=param1,
                                                           param2=param2,
                                                           param3=param3,
                                                           param4=param4,
                                                           param5=param5,
                                                           param6=param6,
                                                           param7=param7)
            self.__vehicle.mav.send(message)

    def request_message_stream(self, message, rate):
        """
        Request a message stream from the vehicle.
        """
        if self.wait_connected():
            self.send_command_long(command=dialect.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   param1=message,
                                   param2=int(1e6 / rate))

    def cancel_message_stream(self, message):
        """
        Cancel a message stream from the vehicle.
        """
        if self.wait_connected():
            self.send_command_long(command=dialect.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   param1=message,
                                   param2=-1)

    def arm(self):
        """
        Arm the vehicle.
        """
        if self.wait_connected():
            self.send_command_long(command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
                                   param1=1)

    def wait_armed(self, timeout=10):
        """
        Wait for the vehicle to arm.
        """
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.armed:
                return True
            time.sleep(0.1)

    def disarm(self):
        """
        Disarm the vehicle.
        """
        if self.wait_connected():
            self.send_command_long(command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
                                   param1=0)

    def wait_disarmed(self, timeout=10):
        """
        Wait for the vehicle to disarm.
        """
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.disarmed:
                return True
            time.sleep(0.1)

    def mode_is_supported(self, mode):
        """
        Check if the vehicle supports a flight mode.
        """
        return mode in self.available_modes

    def change_mode(self, mode):
        """
        Change the vehicle flight mode.
        """
        if self.mode_is_supported(mode=mode) and self.wait_connected():
            self.send_command_long(command=dialect.MAV_CMD_DO_SET_MODE,
                                   param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                   param2=self.__flight_modes[mode])

    def wait_mode(self, mode, timeout=10):
        """
        Wait for the vehicle to change flight mode.
        """
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.mode == mode:
                return True
            time.sleep(0.1)

    def get_rc_channel(self, channel):
        """
        Return an RC channel value.
        """
        return self.get_rc_channels[channel - 1]

    def get_servo_output(self, servo):
        """
        Return a servo value.
        """
        return self.get_rc_channels[servo - 1]

    def set_rc_channel(self, channel, value):
        """
        Set an RC channel value.
        """
        if self.wait_connected():
            temp_channels = [65535] * 18
            temp_channels[channel - 1] = value
            message = dialect.MAVLink_rc_channels_override_message(self.__vehicle.target_system,
                                                                   self.__vehicle.target_component,
                                                                   *temp_channels)
            self.__vehicle.mav.send(message)

    def wait_rc_channel(self, channel, value, timeout=10):
        """
        Wait for an RC channel to change value.
        """
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.get_rc_channel(channel=channel) == value:
                return True
            time.sleep(0.1)

    def set_rc_channels(self, channels_and_values):
        """
        Set RC channel values.
        """
        if self.wait_connected():
            temp_channels = [65535] * 18
            for i in range(len(temp_channels)):
                if i + 1 in channels_and_values.keys():
                    temp_channels[i] = channels_and_values[i + 1]
            channels = temp_channels
            message = dialect.MAVLink_rc_channels_override_message(self.__vehicle.target_system,
                                                                   self.__vehicle.target_component,
                                                                   *channels)
            self.__vehicle.mav.send(message)

    def clear_rc_channel(self, channel):
        """
        Release RC overrides.
        """
        if self.wait_connected():
            temp_channels = [65535] * 18
            if channel < 9:
                temp_channels[channel - 1] = 0
            else:
                temp_channels[channel - 1] = 65534
            message = dialect.MAVLink_rc_channels_override_message(self.__vehicle.target_system,
                                                                   self.__vehicle.target_component,
                                                                   *temp_channels)
            self.__vehicle.mav.send(message)

    def clear_rc_channels(self, channels):
        """
        Release RC overrides.
        """
        if self.wait_connected():
            temp_channels = [65535] * 18
            for channel in channels:
                if channel < 9:
                    temp_channels[channel - 1] = 0
                else:
                    temp_channels[channel - 1] = 65534
            message = dialect.MAVLink_rc_channels_override_message(self.__vehicle.target_system,
                                                                   self.__vehicle.target_component,
                                                                   *temp_channels)
            self.__vehicle.mav.send(message)

    def clear_all_rc_channels(self):
        """
        Release all RC overrides.
        """
        if self.wait_connected():
            temp_channels = [0] * 8 + [65534] * 10
            message = dialect.MAVLink_rc_channels_override_message(self.__vehicle.target_system,
                                                                   self.__vehicle.target_component,
                                                                   *temp_channels)
            self.__vehicle.mav.send(message)

    def takeoff(self, altitude=10):
        """
        Takeoff the vehicle.
        """
        if self.wait_connected():
            self.send_command_long(command=dialect.MAV_CMD_NAV_TAKEOFF,
                                   param7=altitude)

    def land(self):
        """
        Land the vehicle.
        """
        if self.wait_connected():
            self.send_command_long(command=dialect.MAV_CMD_NAV_LAND)

    def reached_altitude(self, altitude, absolute=False, precision=1.0):
        if absolute:
            if abs(self.absolute_altitude - altitude) <= precision:
                return True
        else:
            if abs(self.relative_altitude - altitude) <= precision:
                return True
        return False

    def wait_altitude(self, altitude, timeout=10, absolute=False, precision=1.0):
        """
        Wait for the vehicle to reach an altitude.
        """
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.reached_altitude(altitude=altitude, absolute=absolute, precision=precision):
                return True
            time.sleep(0.1)

    def fly_to(self, latitude, longitude, altitude, absolute=False):
        """
        Fly to a position.
        """
        if self.wait_connected():
            message = dialect.MAVLink_mission_item_int_message(target_system=self.__vehicle.target_system,
                                                               target_component=self.__vehicle.target_component,
                                                               seq=0,
                                                               frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                               command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                               current=2,
                                                               autocontinue=0,
                                                               param1=0,
                                                               param2=0,
                                                               param3=0,
                                                               param4=0,
                                                               x=int(latitude * 1e7),
                                                               y=int(longitude * 1e7),
                                                               z=altitude)
            if absolute:
                message.frame = dialect.MAV_FRAME_GLOBAL_INT
            self.__vehicle.mav.send(message)

    def reached_location(self, latitude, longitude, precision=1.0):
        """
        Check if vehicle is at the location.
        """
        return geopy.distance.distance((self.latitude, self.longitude), (latitude, longitude)).meters <= precision

    def wait_location_2d(self, latitude, longitude, timeout=10, precision=1.0):
        """
        Wait for the vehicle to reach a 2D location.
        """
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.reached_location(latitude=latitude, longitude=longitude, precision=precision):
                return True
            time.sleep(0.1)

    def wait_location_3d(self, latitude, longitude, altitude, absolute=False, timeout=10, precision=1.0):
        """
        Wait for the vehicle to reach a 3D location.
        """
        start_time = time.monotonic()
        while True:
            if time.monotonic() - start_time > timeout > 0:
                return False
            if self.reached_location(latitude=latitude, longitude=longitude, precision=precision) and \
                    self.reached_altitude(altitude=altitude, absolute=absolute, precision=precision):
                return True
            time.sleep(0.1)

    @property
    def connected(self):
        """
        Return if the vehicle is connected.
        """
        return self.__connected

    @property
    def ready(self):
        """
        Return if the vehicle is ready.
        """
        return self.__ready

    @property
    def messages(self):
        """
        Return the vehicle messages.
        """
        return self.__messages

    @property
    def parameters(self):
        """
        Return the vehicle parameters.
        """
        return {parameter["param_id"]: parameter["param_value"] for parameter in list(self.__parameters.values())}

    @property
    def armed(self):
        """
        Return if the vehicle is armed.
        """
        if not self.__connected:
            return None
        base_mode = self.__messages.get("HEARTBEAT", {}).get("base_mode", 0)
        return base_mode & dialect.MAV_MODE_FLAG_SAFETY_ARMED == dialect.MAV_MODE_FLAG_SAFETY_ARMED

    @property
    def disarmed(self):
        """
        Return if the vehicle is disarmed.
        """
        if not self.__connected:
            return None
        return not self.armed

    @property
    def armable(self):
        """
        Return if the vehicle is able to arm.
        """
        if not self.__connected:
            return None
        sys_status = self.__messages.get("SYS_STATUS", {})
        if sys_status == {}:
            return None
        onboard_control_sensors_health = sys_status["onboard_control_sensors_health"]
        pre_arm_status_bit = onboard_control_sensors_health & dialect.MAV_SYS_STATUS_PREARM_CHECK
        pre_arm_status = pre_arm_status_bit == dialect.MAV_SYS_STATUS_PREARM_CHECK
        return pre_arm_status

    @property
    def mode(self):
        """
        Return the vehicle flight mode.
        """
        if not self.__connected:
            return None
        custom_mode = self.__messages.get("HEARTBEAT", {}).get("custom_mode", 0)
        return list(filter(lambda x: self.__flight_modes.get(x, 0) == custom_mode, self.__flight_modes))[0]

    @property
    def available_modes(self):
        """
        Return the available flight modes.
        """
        if not self.__connected:
            return []
        return list(self.__flight_modes.keys())

    @property
    def latitude(self):
        """
        Return the vehicle latitude in degrees.
        """
        return self.__messages.get("GLOBAL_POSITION_INT", {}).get("lat", 0) * 1e-7

    @property
    def longitude(self):
        """
        Return the vehicle longitude in degrees.
        """
        return self.__messages.get("GLOBAL_POSITION_INT", {}).get("lon", 0) * 1e-7

    @property
    def absolute_altitude(self):
        """
        Return the vehicle absolute altitude (MSL) in meters.
        """
        if "GLOBAL_POSITION_INT" in self.__messages:
            return self.__messages.get("GLOBAL_POSITION_INT", {}).get("alt", 0) * 1e-3
        return self.__messages.get("VFR_HUD", {}).get("alt", 0)

    @property
    def relative_altitude(self):
        """
        Return the vehicle relative altitude (above home) in meters.
        """
        return self.__messages.get("GLOBAL_POSITION_INT", {}).get("relative_alt", 0) * 1e-3

    @property
    def yaw(self):
        """
        Return the vehicle yaw in degrees.
        """
        return math.degrees(self.__messages.get("ATTITUDE", {}).get("yaw", 0))

    @property
    def heading(self):
        """
        Return the vehicle heading in degrees.
        """
        if "VFR_HUD" in self.__messages:
            return self.__messages.get("VFR_HUD", {}).get("heading", 0)
        elif "GLOBAL_POSITION_INT" in self.__messages:
            return self.__messages.get("GLOBAL_POSITION_INT", {}).get("hdg", 0) * 1e-2
        return (360 + self.yaw) % 360

    @property
    def ground_speed_north(self):
        """
        Return the vehicle ground speed north in meters per second.
        """
        if "GLOBAL_POSITION_INT" in self.__messages:
            return self.__messages.get("GLOBAL_POSITION_INT", {}).get("vx", 0) * 1e-2
        return self.__messages.get("LOCAL_POSITION_NED", {}).get("vx", 0)

    @property
    def ground_speed_east(self):
        """
        Return the vehicle ground speed east in meters per second.
        """
        if "GLOBAL_POSITION_INT" in self.__messages:
            return self.__messages.get("GLOBAL_POSITION_INT", {}).get("vy", 0) * 1e-2
        return self.__messages.get("LOCAL_POSITION_NED", {}).get("vy", 0)

    @property
    def ground_speed(self):
        """
        Return the vehicle ground speed in meters per second.
        """
        if "GLOBAL_POSITION_INT" in self.__messages:
            return (self.ground_speed_north ** 2 + self.ground_speed_east ** 2) ** 0.5
        return self.__messages.get("VFR_HUD", {}).get("groundspeed", 0)

    @property
    def climb_rate(self):
        """
        Return the vehicle climb rate in meters per second.
        """
        if "GLOBAL_POSITION_INT" in self.__messages:
            return self.__messages.get("GLOBAL_POSITION_INT", {}).get("vz", 0) * 1e-2 * -1
        elif "LOCAL_POSITION_NED" in self.__messages:
            return self.__messages.get("LOCAL_POSITION_NED", {}).get("vz", 0) * -1
        return self.__messages.get("VFR_HUD", {}).get("climb", 0)

    @property
    def air_speed(self):
        """
        Return the vehicle air speed in meters per second.
        """
        return self.__messages.get("VFR_HUD", {}).get("airspeed", 0)

    @property
    def throttle(self):
        """
        Return the vehicle  output throttle level in percent.
        """
        return self.__messages.get("VFR_HUD", {}).get("throttle", 0)

    @property
    def roll(self):
        """
        Return the vehicle roll in degrees.
        """
        return math.degrees(self.__messages.get("ATTITUDE", {}).get("roll", 0))

    @property
    def pitch(self):
        """
        Return the vehicle pitch in degrees.
        """
        return math.degrees(self.__messages.get("ATTITUDE", {}).get("pitch", 0))

    @property
    def roll_speed(self):
        """
        Return the vehicle roll speed in degrees per second.
        """
        return math.degrees(self.__messages.get("ATTITUDE", {}).get("rollspeed", 0))

    @property
    def pitch_speed(self):
        """
        Return the vehicle pitch speed in degrees per second.
        """
        return math.degrees(self.__messages.get("ATTITUDE", {}).get("pitchspeed", 0))

    @property
    def yaw_speed(self):
        """
        Return the vehicle yaw speed in degrees per second.
        """
        return math.degrees(self.__messages.get("ATTITUDE", {}).get("yawspeed", 0))

    @property
    def x(self):
        """
        Return the vehicle x position in meters.
        """
        return self.__messages.get("LOCAL_POSITION_NED", {}).get("x", 0)

    @property
    def y(self):
        """
        Return the vehicle y position in meters.
        """
        return self.__messages.get("LOCAL_POSITION_NED", {}).get("y", 0)

    @property
    def z_up(self):
        """
        Return the vehicle z position (up positive) in meters.
        """
        return self.__messages.get("LOCAL_POSITION_NED", {}).get("z", 0) * -1

    @property
    def home_latitude(self):
        """
        Return the vehicle home latitude in degrees.
        """
        return self.__messages.get("HOME_POSITION", {}).get("latitude", 0) * 1e-7

    @property
    def home_longitude(self):
        """
        Return the vehicle home longitude in degrees.
        """
        return self.__messages.get("HOME_POSITION", {}).get("longitude", 0) * 1e-7

    @property
    def home_absolute_altitude(self):
        """
        Return the vehicle home absolute altitude (MSL) in meters.
        """
        return self.__messages.get("HOME_POSITION", {}).get("altitude", 0) * 1e-3

    @property
    def home_x(self):
        """
        Return the vehicle home x position in meters.
        """
        return self.__messages.get("HOME_POSITION", {}).get("x", 0)

    @property
    def home_y(self):
        """
        Return the vehicle home y position in meters.
        """
        return self.__messages.get("HOME_POSITION", {}).get("y", 0)

    @property
    def home_z_up(self):
        """
        Return the vehicle home z position (up positive) in meters.
        """
        return self.__messages.get("HOME_POSITION", {}).get("z", 0) * -1

    @property
    def home_distance_2d(self):
        """
        Return the vehicle 2D home distance in meters.
        """
        return geopy.distance.distance((self.latitude, self.longitude),
                                       (self.home_latitude, self.home_longitude)).meters

    @property
    def home_distance_3d(self):
        """
        Return the vehicle 3D home distance in meters.
        """
        return (self.home_distance_2d ** 2 + (self.absolute_altitude - self.home_absolute_altitude) ** 2) ** 0.5

    @property
    def origin_distance_2d(self):
        """
        Return the vehicle 2D origin distance in meters.
        """
        return (self.x ** 2 + self.y ** 2) ** 0.5

    @property
    def origin_distance_3d(self):
        """
        Return the vehicle 3D origin distance in meters.
        """
        return (self.origin_distance_2d ** 2 + self.z_up ** 2) ** 0.5

    @property
    def target_latitude(self):
        """
        Return the vehicle target position latitude in degrees.
        """
        return self.__messages.get("POSITION_TARGET_GLOBAL_INT", {}).get("lat_int", 0) * 1e-7

    @property
    def target_longitude(self):
        """
        Return the vehicle target position longitude in degrees.
        """
        return self.__messages.get("POSITION_TARGET_GLOBAL_INT", {}).get("lon_int", 0) * 1e-7

    @property
    def target_absolute_altitude(self):
        """
        Return the vehicle target position absolute altitude (MSL) in meters.
        """
        return self.__messages.get("POSITION_TARGET_GLOBAL_INT", {}).get("alt", 0)

    @property
    def target_x(self):
        """
        Return the vehicle target position x in meters.
        """
        return self.__messages.get("POSITION_TARGET_LOCAL_NED", {}).get("x", 0)

    @property
    def target_y(self):
        """
        Return the vehicle target position y in meters.
        """
        return self.__messages.get("POSITION_TARGET_LOCAL_NED", {}).get("y", 0)

    @property
    def target_z_up(self):
        """
        Return the vehicle target position z in meters.
        """
        return self.__messages.get("POSITION_TARGET_LOCAL_NED", {}).get("z", 0) * -1

    @property
    def global_target_distance_2d(self):
        """
        Return the vehicle 2D target distance in meters.
        """
        return geopy.distance.distance((self.latitude, self.longitude),
                                       (self.target_latitude, self.target_longitude)).meters

    @property
    def global_target_distance_3d(self):
        """
        Return the vehicle 3D target distance in meters.
        """
        return (self.global_target_distance_2d ** 2 + (
                self.absolute_altitude - self.target_absolute_altitude) ** 2) ** 0.5

    @property
    def local_target_distance_2d(self):
        """
        Return the vehicle 2D target distance in meters.
        """
        return (self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2

    @property
    def local_target_distance_3d(self):
        """
        Return the vehicle 3D target distance in meters.
        """
        return (self.local_target_distance_2d ** 2 + (self.target_z_up - self.z_up) ** 2) ** 0.5

    @property
    def battery_voltage(self):
        """
        Return the vehicle battery voltage in volts.
        """
        return self.__messages.get("SYS_STATUS", {}).get("voltage_battery", 0) * 1e-3

    @property
    def battery_current(self):
        """
        Return the vehicle battery current in amps.
        """
        return self.__messages.get("SYS_STATUS", {}).get("current_battery", 0) * 1e-2

    @property
    def battery_remaining(self):
        """
        Return the vehicle battery remaining in percent.
        """
        return self.__messages.get("SYS_STATUS", {}).get("battery_remaining", 0)

    @property
    def boot_time(self):
        """
        Return the vehicle boot time in seconds.
        """
        return self.__messages.get("SYSTEM_TIME", {}).get("time_boot_ms", 0) * 1e-3

    @property
    def unix_time(self):
        """
        Return the unix time in seconds.
        """
        return self.__messages.get("SYSTEM_TIME", {}).get("time_unix_usec", 0) * 1e-6

    @property
    def date_time(self):
        """
        Return the date time of the vehicle.
        """
        return datetime.datetime.fromtimestamp(self.unix_time)

    @property
    def gps_fix(self):
        """
        Return the vehicle GPS fix.
        """
        return self.__messages.get("GPS_RAW_INT", {}).get("fix_type", 0)

    @property
    def visible_satellites(self):
        """
        Return the vehicle visible satellites.
        """
        return self.__messages.get("GPS_RAW_INT", {}).get("satellites_visible", 0)

    @property
    def gps_hdop(self):
        """
        Return the vehicle GPS HDOP.
        """
        return self.__messages.get("GPS_RAW_INT", {}).get("eph", 0) * 1e-2

    @property
    def gps_vdop(self):
        """
        Return the vehicle GPS VDOP.
        """
        return self.__messages.get("GPS_RAW_INT", {}).get("epv", 0) * 1e-2

    @property
    def get_rc_channels(self):
        """
        Return the vehicle RC inputs.
        """
        message = self.__messages.get("RC_CHANNELS", {})
        return [message.get(f"chan{i}_raw", 0) for i in range(1, 19)]

    @property
    def get_servo_outputs(self):
        """
        Return the vehicle servo outputs.
        """
        message = self.__messages.get("SERVO_OUTPUT_RAW", {})
        return [message.get(f"servo{i}_raw", 0) for i in range(1, 17)]

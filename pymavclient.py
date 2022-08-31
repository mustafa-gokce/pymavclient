import enum
import time
import math
import requests
import threading


class MAVLINK(enum.Enum):
    MAV_CMD_REQUEST_MESSAGE = 512
    HOME_POSITION = 242


class Speed:
    """
    Speed class.
    """

    def __init__(self):
        self.air = 0.0
        """
        Air speed in meters per second.
        """

        self.ground = 0.0
        """
        Ground speed in meters per second.
        """

        self.climb = 0.0
        """
        Climb rate in meters per second.
        """

        self.north = 0.0
        """
        Ground north positive speed in meters per second.
        """

        self.east = 0.0
        """
        Ground east positive speed in meters per second.
        """

        self.up = 0.0
        """
        Ground up positive speed in meters per second.
        """

        self.x = 0.0
        """
        Local x positive speed in meters per second.
        """

        self.y = 0.0
        """
        Local y positive speed in meters per second.
        """

        self.z_up = 0.0
        """
        Local up positive speed in meters per second.
        """

        self.roll = 0.0
        """
        Rotation speed on roll axis degrees per second.
        """

        self.pitch = 0.0
        """
        Rotation speed on pitch axis degrees per second.
        """

        self.yaw = 0.0
        """
        Rotation speed on yaw axis degrees per second.
        """

    def __dict__(self):
        """
        Speed class dictionary.

        :return: dict
        """

        # get speed class as dictionary
        return {"air": self.air,
                "ground": self.ground,
                "climb": self.climb,
                "north": self.north,
                "east": self.east,
                "up": self.up,
                "x": self.x,
                "y": self.y,
                "z_up": self.z_up,
                "roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw}

    def __str__(self):
        """
        Speed class dictionary as string.

        :return: str
        """

        # get speed class dictionary as string
        return str(self.__dict__())


class Location:
    """
    Location class.
    """

    def __init__(self):
        self.latitude = 0.0
        """
        Latitude in degrees.
        """

        self.longitude = 0.0
        """
        Longitude in degrees.
        """

        self.altitude_msl = 0.0
        """
        Altitude above mean sea level in meters.
        """

        self.altitude_relative = 0.0
        """
        Altitude above home in meters.
        """

        self.x = 0.0
        """
        Position x in meters.
        """

        self.y = 0.0
        """
        Position y in meters.
        """

        self.z_up = 0.0
        """
        Position z up positive in meters.
        """

    def __dict__(self):
        """
        Location class dictionary.

        :return: dict
        """

        # get location class as dictionary
        return {"latitude": self.latitude,
                "longitude": self.longitude,
                "altitude_relative": self.altitude_relative,
                "altitude_msl": self.altitude_msl,
                "x": self.x,
                "y": self.y,
                "z_up": self.z_up}

    def __str__(self):
        """
        Location class dictionary as string.

        :return: str
        """

        # get location class dictionary as string
        return str(self.__dict__())


class Attitude:
    """
    Attitude class.
    """

    def __init__(self):
        self.roll = 0.0
        """
        Roll angle in degrees.
        """

        self.pitch = 0.0
        """
        Pitch angle in degrees.
        """

        self.yaw = 0.0
        """
        Yaw angle in degrees.
        """

        self.heading = 0.0
        """
        Heading in degrees.
        """

    def __dict__(self):
        """
        Attitude class dictionary.

        :return: dict
        """

        # get attitude class as dictionary
        return {"roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw,
                "heading": self.heading}

    def __str__(self):
        """
        Attitude class dictionary as string.

        :return: str
        """

        # get attitude class dictionary as string
        return str(self.__dict__())


class Distance:
    """
    Distance class.
    """

    def __init__(self):
        self.home_2d = 0.0
        """
        Home distance 2d in meters.
        """

        self.home_3d = 0.0
        """
        Home distance 3d in meters.
        """

        self.origin_2d = 0.0
        """
        Origin distance 2d in meters.
        """

        self.origin_3d = 0.0
        """
        Origin distance 3d in meters.
        """

        self.target_2d = 0.0
        """
        Target distance 2d in meters.
        """

        self.target_3d = 0.0
        """
        Target distance 3d in meters.
        """

    def __dict__(self):
        """
        Distance class dictionary.

        :return: dict
        """

        # get distance class as dictionary
        return {"home_2d": self.home_2d,
                "home_3d": self.home_3d,
                "origin_2d": self.origin_2d,
                "origin_3d": self.origin_3d,
                "target_2d": self.target_2d,
                "target_3d": self.target_3d}

    def __str__(self):
        """
        Distance class dictionary as string.

        :return: str
        """

        # get distance class dictionary as string
        return str(self.__dict__())


class MissionFenceRally:
    """
    Mission/Fence/Rally class.
    """

    def __init__(self):
        self.total = 0
        """
        Total mission/fence/rally item count.
        """

        self.items = []
        """
        Mission/Fence/Rally items.
        """

    def __dict__(self):
        """
        Mission/Fence/Rally class dictionary.

        :return: dict
        """

        # get mission/fence/rally class as dictionary
        return {"total": self.total,
                "items": self.items}

    def __str__(self):
        """
        Mission/Fence/Rally class dictionary as string.

        :return: str
        """

        # get mission/fence/rally class dictionary as string
        return str(self.__dict__())


class Mission(MissionFenceRally):
    """
    Mission class.
    """

    def __init__(self):
        super().__init__()

        self.current = 0
        """
        Current mission item sequence number.
        """

    def __dict__(self):
        """
        Mission class dictionary.

        :return: dict
        """

        # get mission class as dictionary
        return {**super().__dict__(), **{"current": self.current}}

    def __str__(self):
        """
        Mission class dictionary as string.

        :return: str
        """

        # get mission class dictionary as string
        return str(self.__dict__())


class Fence(MissionFenceRally):
    """
    Fence class.
    """

    def __init__(self):
        super().__init__()


class Rally(MissionFenceRally):
    """
    Rally class.
    """

    def __init__(self):
        super().__init__()


class Vehicle:
    """
    Vehicle class.
    """

    def __init__(self):
        self.location = Location()
        """
        Location of the vehicle.
        """

        self.attitude = Attitude()
        """
        Attitude of the vehicle.
        """

        self.speed = Speed()
        """
        Speed of the vehicle.
        """

        self.home = Location()
        """
        Home location of the vehicle.
        """

        self.distance = Distance()
        """
        Distance of the vehicle.
        """

        self.mission = Mission()
        """
        Mission of the vehicle.
        """

        self.fence = Fence()
        """
        Fence of the vehicle.
        """

        self.rally = Rally()
        """
        Rally of the vehicle.
        """

        self.parameters = {}
        """
        Parameters of the vehicle.
        """

    def __dict__(self):
        """
        Vehicle class dictionary.

        :return: dict
        """

        # get vehicle class as dictionary
        return {"location": self.location.__dict__(),
                "attitude": self.attitude.__dict__(),
                "speed": self.speed.__dict__(),
                "home": self.home.__dict__(),
                "distance": self.distance.__dict__(),
                "mission": self.mission.__dict__(),
                "fence": self.fence.__dict__(),
                "rally": self.rally.__dict__(),
                "parameters": self.parameters}

    def __str__(self):
        """
        Vehicle class dictionary as string.

        :return: str
        """

        # get vehicle class dictionary as string
        return str(self.__dict__())


class PyMAVClient:
    """
    PyMAVClient class.
    """

    def __init__(self, ip="127.0.0.1", port=2609, run=True,
                 delay_vehicle=0.25, delay_message=0.25, delay_parameter=5.0,
                 delay_plan=5.0, delay_fence=5.0, delay_rally=5.0, request_home=True):
        self.ip = ip
        """
        IP address of the server.
        """

        self.port = port
        """
        Port of the server.
        """

        self._message_all = {}
        """
        All messages from server.
        """

        self._parameter_all = {}
        """
        All parameters from server.
        """

        self._plan_all = []
        """
        Entire plan from server.
        """

        self._fence_all = []
        """
        Entire fence from server.
        """

        self._rally_all = []
        """
        Entire rally from server.
        """

        self.vehicle = Vehicle()
        """
        Vehicle of the server.
        """

        self.run = run
        """
        Start communicating to server.
        """

        self.delay_vehicle = delay_vehicle
        """
        Delay between vehicle updates in seconds.
        """

        self.delay_message = delay_message
        """
        Delay between message updates in seconds.
        """

        self.delay_parameter = delay_parameter
        """
        Delay between parameter updates in seconds.
        """

        self.delay_plan = delay_plan
        """
        Delay between plan updates in seconds.
        """

        self.delay_fence = delay_fence
        """
        Delay between fence updates in seconds.
        """

        self.delay_rally = delay_rally
        """
        Delay between rally updates in seconds.
        """

        self.request_home = request_home
        """
        Request home location from server.
        """

        # helper stop flags and threads
        self._thread_vehicle_stop = False
        self._thread_message_stop = False
        self._thread_parameter_stop = False
        self._thread_plan_stop = False
        self._thread_fence_stop = False
        self._thread_rally_stop = False
        self._thread_home_stop = False
        self._thread_vehicle = None
        self._thread_message = None
        self._thread_parameter = None
        self._thread_plan = None
        self._thread_fence = None
        self._thread_rally = None
        self._thread_home = None

        # connect to server if requested
        if self.run:
            self.start()

        # get home location if requested
        if self.request_home:
            self._thread_home = self._create_thread(target=self.get_home, delay=5.0,
                                                    fresh=True, stop=self._thread_home_stop)

    def _infinite_loop(self, target, delay, fresh, stop):
        """
        Infinite loop.

        :param target: Function to run in the loop.
        :param delay: Delay between each loop.
        :param fresh: Fresh data.

        :return:
        """
        while True:
            if stop:
                break
            target(fresh=fresh)
            time.sleep(delay)

    def _create_thread(self, target, delay, fresh, stop):
        """
        Create a thread and start it.

        :param target: Function to run in the thread.
        :param delay: Delay between each loop.
        :param fresh: Fresh data.
        :param stop: Stop thread flag.

        :return:
        """
        thread = threading.Thread(target=self._infinite_loop, args=(target, delay, fresh, stop))
        thread.daemon = True
        thread.start()
        return thread

    def start(self):
        """
        Start communicating to server.
        """

        # start threads
        self._thread_vehicle_stop = False
        self._thread_message_stop = False
        self._thread_parameter_stop = False
        self._thread_plan_stop = False
        self._thread_fence_stop = False
        self._thread_rally_stop = False
        self._thread_vehicle = self._create_thread(target=self.vehicle_update, delay=self.delay_vehicle,
                                                   fresh=False, stop=self._thread_vehicle_stop)
        self._thread_message = self._create_thread(target=self.get_message_all, delay=self.delay_message,
                                                   fresh=True, stop=self._thread_message_stop)
        self._thread_parameter = self._create_thread(target=self.get_parameter_all, delay=self.delay_parameter,
                                                     fresh=True, stop=self._thread_parameter_stop)
        self._thread_plan = self._create_thread(target=self.get_plan_item_all, delay=self.delay_plan,
                                                fresh=True, stop=self._thread_plan_stop)
        self._thread_fence = self._create_thread(target=self.get_fence_item_all, delay=self.delay_fence,
                                                 fresh=True, stop=self._thread_fence_stop)
        self._thread_rally = self._create_thread(target=self.get_rally_item_all, delay=self.delay_rally,
                                                 fresh=True, stop=self._thread_rally_stop)

    def stop(self):
        """
        Stop communicating to server.
        """

        # stop threads
        self._thread_vehicle_stop = False
        self._thread_message_stop = False
        self._thread_parameter_stop = False
        self._thread_plan_stop = False
        self._thread_fence_stop = False
        self._thread_rally_stop = False
        self._thread_home_stop = False

        # clear threads
        self._thread_vehicle = None
        self._thread_message = None
        self._thread_parameter = None
        self._thread_plan = None
        self._thread_fence = None
        self._thread_rally = None
        self._thread_home = None

    @property
    def url(self):
        """
        Server url.

        :return:
        """
        return f"http://{self.ip}:{self.port}"

    @property
    def url_get(self):
        """
        Server get url.

        :return:
        """
        return f"{self.url}/get"

    def url_get_message(self, message_name):
        """
        Server message get url.

        :param message_name: Name of the message.

        :return:
        """
        return f"{self.url_get}/message/{message_name}"

    @property
    def url_get_message_all(self):
        """
        Server message get all url.

        :return:
        """
        return self.url_get_message(message_name="all")

    def url_get_parameter(self, parameter_name):
        """
        Server parameter get url.

        :param parameter_name: Name of the parameter.

        :return:
        """
        return f"{self.url_get}/parameter/{parameter_name}"

    @property
    def url_get_parameter_all(self):
        """
        Server parameter get all url.

        :return:
        """
        return self.url_get_parameter(parameter_name="all")

    def url_get_plan_item(self, item_index):
        """
        Server plan item get url.

        :param item_index: Index of the plan item.

        :return:
        """
        return f"{self.url_get}/plan/{item_index}"

    @property
    def url_get_plan_item_all(self):
        """
        Server plan get url.

        :return:
        """
        return self.url_get_plan_item(item_index="all")

    def url_get_rally_item(self, item_index):
        """
        Server rally item get url.

        :param item_index: Index of the rally item.

        :return:
        """
        return f"{self.url_get}/rally/{item_index}"

    @property
    def url_get_rally_item_all(self):
        """
        Server rally get url.

        :return:
        """
        return self.url_get_rally_item(item_index="all")

    def url_get_fence_item(self, item_index):
        """
        Server fence item get url.

        :param item_index: Index of the fence item.

        :return:
        """
        return f"{self.url_get}/fence/{item_index}"

    @property
    def url_get_fence_item_all(self):
        """
        Server fence get url.

        :return:
        """
        return self.url_get_fence_item(item_index="all")

    @property
    def url_post(self):
        """
        Server get url.

        :return:
        """
        return f"{self.url}/post"

    @property
    def url_post_command_long(self):
        """
        Server post command long url.

        :return:
        """
        return f"{self.url_post}/command_long"

    @staticmethod
    def get(url):
        """
        Get data from server.

        :param url: URL to get data from.

        :return:
        """
        try:
            result = requests.get(url).json()
        except Exception as e:
            result = None
        return result

    @staticmethod
    def post(url, json):
        """
        Post data to server.

        :param url: URL to post data.
        :param json: Data to post.

        :return:
        """
        try:
            result = requests.post(url=url, json=json).json()
        except Exception as e:
            result = None
        return result

    def get_message_all(self, fresh=False):
        """
        Get all messages from vehicle.

        :param fresh: Get messages from server, do not use the cached messages.

        :return:
        """
        message_all = self._message_all
        if fresh:
            data = self.get(url=self.url_get_message_all)
            if data not in ({}, [], None):
                message_all = data
                self._message_all = {**self._message_all, **message_all}
        return message_all

    def get_message(self, message_name, fresh=False):
        """
        Get the requested message from vehicle.

        :param fresh: Get message from server, do not use the cached message.
        :param message_name: Message name.

        :return:
        """

        message = None
        if message_name in self._message_all.keys() and not fresh:
            message = self._message_all[message_name]
        else:
            data = self.get(url=self.url_get_message(message_name=message_name))
            if data not in ({}, [], None):
                message = data
                self._message_all = {**self._message_all, **message}
        return message

    def get_parameter_all(self, fresh=False):
        """
        Get all parameters from vehicle.

        :param fresh: Get parameters from server, do not use the cached parameters.

        :return:
        """
        parameter_all = self._parameter_all
        if fresh:
            data = self.get(url=self.url_get_parameter_all)
            if data not in ({}, [], None):
                parameter_all = data
                self._parameter_all = {**self._parameter_all, **parameter_all}
        return parameter_all

    def get_parameter(self, parameter_name, fresh=False):
        """
        Get the requested parameter from vehicle.

        :param fresh: Get parameter from server, do not use the cached parameter.
        :param parameter_name: Parameter name.

        :return:
        """

        parameter = None
        if parameter_name in self._parameter_all.keys() and not fresh:
            parameter = self._parameter_all[parameter_name]
        else:
            data = self.get(url=self.url_get_parameter(parameter_name=parameter_name))
            if data not in ({}, [], None):
                parameter = data
                self._parameter_all = {**self._parameter_all, **parameter}
        return parameter

    def get_plan_item_all(self, fresh=False):
        """
        Get entire plan from vehicle.

        :param fresh: Get plan from server, do not use the cached plan.

        :return:
        """
        plan_all = self._plan_all
        if fresh:
            data = self.get(url=self.url_get_plan_item_all)
            if data not in ({}, [], None):
                plan_all = data
                self._plan_all = plan_all
        return plan_all

    def get_plan_item(self, item_index, fresh=False):
        """
        Returns the requested plan item from vehicle.

        :param fresh: Get plan item from server, do not use the cached plan item.
        :param item_index: Index of the plan item.

        :return:
        """

        plan_item = None
        if not fresh:
            for item in self._plan_all:
                if item["seq"] == item_index:
                    plan_item = item
                    break
        if plan_item is None and fresh:
            data = self.get(url=self.url_get_plan_item(item_index=item_index))
            if data not in ({}, [], None):
                plan_item = data
        return plan_item

    def get_rally_item_all(self, fresh=False):
        """
        Get entire rally from vehicle.

        :param fresh: Get rally from server, do not use the cached rally.

        :return:
        """
        rally_all = self._rally_all
        if fresh:
            data = self.get(url=self.url_get_rally_item_all)
            if data not in ({}, [], None):
                rally_all = data
                self._rally_all = rally_all
        return rally_all

    def get_rally_item(self, item_index, fresh=False):
        """
        Returns the requested rally item from vehicle.

        :param fresh: Get rally item from server, do not use the cached rally item.
        :param item_index: Index of the rally item.

        :return:
        """

        rally_item = None
        if not fresh:
            for item in self._rally_all:
                if item["seq"] == item_index:
                    rally_item = item
                    break
        if rally_item is None and fresh:
            data = self.get(url=self.url_get_rally_item(item_index=item_index))
            if data not in ({}, [], None):
                rally_item = data
        return rally_item

    def get_fence_item_all(self, fresh=False):
        """
        Get entire fence from vehicle.

        :param fresh: Get fence from server, do not use the cached fence.

        :return:
        """
        fence_all = self._fence_all
        if fresh:
            data = self.get(url=self.url_get_fence_item_all)
            if data not in ({}, [], None):
                fence_all = data
                self._fence_all = fence_all
        return fence_all

    def get_fence_item(self, item_index, fresh=False):
        """
        Returns the requested fence item from vehicle.

        :param fresh: Get fence item from server, do not use the cached fence item.
        :param item_index: Index of the fence item.

        :return:
        """

        fence_item = None
        if not fresh:
            for item in self._fence_all:
                if item["seq"] == item_index:
                    fence_item = item
                    break
        if fence_item is None and fresh:
            data = self.get(url=self.url_get_fence_item(item_index=item_index))
            if data not in ({}, [], None):
                fence_item = data
        return fence_item

    def get_home(self, fresh=False):
        """
        Get home location from vehicle.

        :param fresh: Get home location from server, do not use the cached home location.

        :return:
        """

        # if home is not set get from server and cache it
        if self.vehicle.home.latitude == 0.0 and self.vehicle.home.longitude == 0.0:
            fresh = True

        # home is populated so stop the home request thread
        else:
            self._thread_home_stop = True
            self._thread_home = None

        # needed to request from server
        if fresh:
            # get home location from server
            self.post_command_long(command=MAVLINK.MAV_CMD_REQUEST_MESSAGE.value,
                                   param1=MAVLINK.HOME_POSITION.value)

        # return home location
        return self.vehicle.home

    def post_command_long(self, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """
        Send a command long message to the vehicle.

        :param command: Command long identifier.
        :param param1: First parameter.
        :param param2: Second parameter.
        :param param3: Third parameter.
        :param param4: Fourth parameter.
        :param param5: Fifth parameter.
        :param param6: Sixth parameter.
        :param param7: Seventh parameter.

        :return:
        """

        # create command long message
        json = {"target_system": 0,
                "target_component": 0,
                "command": command,
                "confirmation": 0,
                "param1": param1,
                "param2": param2,
                "param3": param3,
                "param4": param4,
                "param5": param5,
                "param6": param6,
                "param7": param7}

        # send command long message to server
        result = self.post(url=self.url_post_command_long, json=json)

        # return to server response
        return result

    def vehicle_update(self, fresh=False):
        """
        Update vehicle.

        :param fresh: Update vehicle from server, do not use the cached messages.

        :return:
        """

        # get messages
        message_all = self.get_message_all(fresh=fresh)
        if message_all not in ({}, [], None):
            message_keys = message_all.keys()

            # use VFR_HUD message to update vehicle
            if "VFR_HUD" in message_keys:
                self.vehicle.location.altitude_msl = message_all["VFR_HUD"]["alt"]
                self.vehicle.attitude.heading = message_all["VFR_HUD"]["heading"]
                self.vehicle.speed.air = message_all["VFR_HUD"]["airspeed"]
                self.vehicle.speed.ground = message_all["VFR_HUD"]["groundspeed"]
                self.vehicle.speed.climb = message_all["VFR_HUD"]["climb"]

            # use GLOBAL_POSITION_INT message to update vehicle
            if "GLOBAL_POSITION_INT" in message_keys:
                self.vehicle.location.latitude = message_all["GLOBAL_POSITION_INT"]["lat"] * 1e-7
                self.vehicle.location.longitude = message_all["GLOBAL_POSITION_INT"]["lon"] * 1e-7
                self.vehicle.location.altitude_msl = message_all["GLOBAL_POSITION_INT"]["alt"] * 1e-3
                self.vehicle.location.altitude_relative = message_all["GLOBAL_POSITION_INT"]["relative_alt"] * 1e-3
                self.vehicle.attitude.heading = message_all["GLOBAL_POSITION_INT"]["hdg"] * 1e-2
                self.vehicle.speed.north = message_all["GLOBAL_POSITION_INT"]["vx"] * 1e-2
                self.vehicle.speed.east = message_all["GLOBAL_POSITION_INT"]["vy"] * 1e-2
                self.vehicle.speed.up = message_all["GLOBAL_POSITION_INT"]["vz"] * -1e-2

            # use ATTITUDE message to update vehicle
            if "ATTITUDE" in message_keys:
                self.vehicle.attitude.roll = math.degrees(message_all["ATTITUDE"]["roll"])
                self.vehicle.attitude.pitch = math.degrees(message_all["ATTITUDE"]["pitch"])
                self.vehicle.attitude.yaw = math.degrees(message_all["ATTITUDE"]["yaw"])
                self.vehicle.speed.roll = math.degrees(message_all["ATTITUDE"]["rollspeed"])
                self.vehicle.speed.pitch = math.degrees(message_all["ATTITUDE"]["pitchspeed"])
                self.vehicle.speed.yaw = math.degrees(message_all["ATTITUDE"]["yawspeed"])

            # use LOCAL_POSITION_NED message to update vehicle
            if "LOCAL_POSITION_NED" in message_keys:
                self.vehicle.location.x = message_all["LOCAL_POSITION_NED"]["x"]
                self.vehicle.location.y = message_all["LOCAL_POSITION_NED"]["y"]
                self.vehicle.location.z_up = message_all["LOCAL_POSITION_NED"]["z"] * -1.0
                self.vehicle.speed.x = message_all["LOCAL_POSITION_NED"]["vx"]
                self.vehicle.speed.y = message_all["LOCAL_POSITION_NED"]["vy"]
                self.vehicle.speed.z_up = message_all["LOCAL_POSITION_NED"]["vz"] * -1.0
                self.vehicle.distance.origin_2d = (self.vehicle.location.x ** 2 +
                                                   self.vehicle.location.y ** 2) ** 0.5
                self.vehicle.distance.origin_3d = (self.vehicle.location.x ** 2 +
                                                   self.vehicle.location.y ** 2 +
                                                   self.vehicle.location.z_up ** 2) ** 0.5
                self.vehicle.distance.home_2d = ((self.vehicle.location.x - self.vehicle.home.x) ** 2 +
                                                 (self.vehicle.location.y - self.vehicle.home.y) ** 2) ** 0.5
                self.vehicle.distance.home_3d = ((self.vehicle.location.x - self.vehicle.home.x) ** 2 +
                                                 (self.vehicle.location.y - self.vehicle.home.y) ** 2 +
                                                 (self.vehicle.location.z_up - self.vehicle.home.z_up) ** 2) ** 0.5

            # use HOME_POSITION message to update vehicle
            if "HOME_POSITION" in message_keys:
                self.vehicle.home.latitude = message_all["HOME_POSITION"]["latitude"] * 1e-7
                self.vehicle.home.longitude = message_all["HOME_POSITION"]["longitude"] * 1e-7
                self.vehicle.home.altitude_msl = message_all["HOME_POSITION"]["altitude"] * 1e-3
                self.vehicle.home.x = message_all["HOME_POSITION"]["x"]
                self.vehicle.home.y = message_all["HOME_POSITION"]["y"]
                self.vehicle.home.z_up = message_all["HOME_POSITION"]["z"] * -1.0

            # use MISSION_CURRENT message to update vehicle
            if "MISSION_CURRENT" in message_keys:
                self.vehicle.mission.current = message_all["MISSION_CURRENT"]["seq"]

        # get plan
        plan_all = self.get_plan_item_all(fresh=fresh)
        if plan_all not in ({}, [], None):
            # use plan to update vehicle
            self.vehicle.mission.items = plan_all
            self.vehicle.mission.total = len(plan_all)

        # get rally
        rally_all = self.get_rally_item_all(fresh=fresh)
        if rally_all not in ({}, [], None):
            # use rally to update vehicle
            self.vehicle.rally.items = rally_all
            self.vehicle.rally.total = len(rally_all)

        # get fence
        fence_all = self.get_fence_item_all(fresh=fresh)
        if fence_all not in ({}, [], None):
            # use fence to update vehicle
            self.vehicle.fence.items = fence_all
            self.vehicle.fence.total = len(fence_all)

        # get parameters
        parameter_all = self.get_parameter_all(fresh=fresh)
        if parameter_all not in ({}, [], None):
            # use parameter values to update vehicle
            self.vehicle.parameters = parameter_all

import pathlib
import requests
import subprocess
import tempfile
import platform
import time


class PyMAVSim:
    """
    Just another ArduPilot SITL wrapper
    """

    __links = {
        "copter": {"firmware_link": "https://firmware.ardupilot.org/"
                                    "Copter/stable/SITL_x86_64_linux_gnu/arducopter",
                   "parameter_link": "https://raw.githubusercontent.com/"
                                     "ArduPilot/ardupilot/master/Tools/autotest/default_params/copter.parm"}}

    def __init__(self, vehicle="copter"):
        """
        Initialize PyMAVSim.
        """

        if platform.system() != "Linux":
            raise Exception("PyMAVSim only supports Linux at the moment")
        self.__vehicle = vehicle
        self.__process = None
        self.__path = pathlib.Path(f"{tempfile.gettempdir()}/pymavsim")
        if not self.__path.exists():
            self.__path.mkdir(parents=True, exist_ok=True)

    def __start_copter(self):
        firmware_path = self.__path / PyMAVSim.__links["copter"]["firmware_link"].split("/")[-1]
        if not firmware_path.exists():
            request = requests.get(PyMAVSim.__links["copter"]["firmware_link"])
            with open(firmware_path, "wb") as file:
                file.write(request.content)
        parameter_path = self.__path / PyMAVSim.__links["copter"]["parameter_link"].split("/")[-1]
        if not parameter_path.exists():
            request = requests.get(PyMAVSim.__links["copter"]["parameter_link"])
            with open(parameter_path, "wb") as file:
                file.write(request.content)
        subprocess.Popen(["chmod", "+x", firmware_path],
                         cwd=self.__path,
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.STDOUT).wait()
        if self.__process is None:
            self.__process = subprocess.Popen(
                [firmware_path, "-w", "-S", "--model", "+", "--speedup", "1", "--defaults", parameter_path],
                cwd=self.__path,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT)
        time.sleep(5)
        if self.__process.poll() is not None:
            raise Exception("PyMAVSim failed to start")

    def start(self):
        if self.__vehicle == "copter":
            self.__start_copter()
        else:
            raise Exception("PyMAVSim only supports copter at the moment")

    def close(self):
        if self.__process is not None:
            self.__process.terminate()
            self.__process.wait()
        self.__process = None

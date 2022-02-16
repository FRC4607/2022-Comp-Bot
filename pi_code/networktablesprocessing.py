# Portions copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from distutils.command.config import config
import json
import sys
import networktables
from cscore import CameraServer

CameraServer.enableLogging()

team = None
server = False
configFile = "/boot/frc.json"

ntinst: networktables.NetworkTablesInstance = None

csOutput = None

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

def configureNetworkTables():
    global ntinst
    global csOutput
    readConfig()

    ntinst = networktables.NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    global csOutput
    csOutput = CameraServer.putVideo("Camera Feed", 300, 300)

def getRobotEnabled() -> bool:
    return ntinst.getTable("PiTable").getBoolean("IsRobotEnabled", False)

def pushCameraServer(frame):
    csOutput.putFrame(frame)
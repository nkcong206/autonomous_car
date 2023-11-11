from redis_db import REDIT
import json
from static import STATIC_VAR


def remove_robot_by_id(robot_id):
    robots = REDIT.get("robots")
    if robots is not None:
        data_robot = json.loads(robots)
        for index, robot in enumerate(data_robot):
            if robot["id"] == robot_id:
                del data_robot[index]
                break
        REDIT.set("robots", json.dumps(data_robot))

def remove_robot_by_client_id(client_id):
    robots = REDIT.get("robots")
    if robots is not None:
        data_robot = json.loads(robots)
        for index, robot in enumerate(data_robot):
            if robot["client_id"] == client_id:
                robot_id = data_robot[index]["id"]
                del data_robot[index]
                REDIT.set("robots", json.dumps(data_robot))
                return True, robot_id
    return False, None

def get_client_id_by_robot_id(robot_id):
    robots = REDIT.get("robots")
    if robots is not None:
        data_robot = json.loads(robots)
        for index, robot in enumerate(data_robot):
            if robot["id"] == robot_id:
                return robot["client_id"]
    return ""

def remove_client_view_by_client_id(client_id):
    for robot_id in STATIC_VAR.CLIENT_VIEWS.keys():
        if client_id in STATIC_VAR.CLIENT_VIEWS[robot_id]:
            STATIC_VAR.CLIENT_VIEWS[robot_id].remove(client_id)
            robot_socket_id = get_client_id_by_robot_id(robot_id)
            return True, len(STATIC_VAR.CLIENT_VIEWS[robot_id]), robot_socket_id
    return False, 0, ""

def check_robot_exit_by_id(client_id):
    robots = REDIT.get("robots")
    if robots is not None:
        data_robot = json.loads(robots)
        for index, robot in enumerate(data_robot):
            if robot["client_id"] == client_id:
                return True
    return False

def get_stream_by_robot_id(robot_id):
    robots = REDIT.get("robots")
    if robots is not None:
        data_robot = json.loads(robots)
        for robot in data_robot:
            if robot["id"] == robot_id:
                return robot["stream"]
    return ""

def get_c_id_by_robot_id(robot_id):
    robots = REDIT.get("c_robots")
    if robots is not None:
        data_robot = json.loads(robots)
        for robot in data_robot:
            if robot["id"] == robot_id:
                return robot["client_id"]
    return ""

def remove_client_controler_by_client_id(client_id):
    c_robots = REDIT.get("c_robots")
    if c_robots is not None:
        data_c_robot = json.loads(c_robots)
        for index, robot in enumerate(data_c_robot):
            if robot["client_id"] == client_id:
                print("c_robots client_id : ", client_id, flush=True)
                robot_id = robot["id"]
                del data_c_robot[index]
                REDIT.set("c_robots", json.dumps(data_c_robot))
                return True, robot_id
    return False, None
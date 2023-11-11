import redis
from PyQt5.QtCore import QThread
import os
import time
import json
from connection.SocketClient import SocketIOClient
from module.Static import STATIC_VAR

class LoadRoBot(QThread):
    def __init__(self, comboBox):
        super(LoadRoBot, self).__init__()
        self.comboBox = comboBox
        self.HOST = os.getenv("SERVER_REDIT", "")
        self.PORT = os.getenv("REDIT_PORT", 6379)
        self.PASS = os.getenv("REDIT_PASSWORD", "")

        self.comboBox.activated.connect(self.add_event)
        self.last_text = None

    def add_event(self):
        robot_id = self.comboBox.currentText()
        if self.last_text:
            stop_stream = {"robot_id" : self.last_text}
            SocketIOClient.get_instance().emit(STATIC_VAR.STOP_STREAM_API, stop_stream)
        start_stream = {"robot_id" : robot_id}
        SocketIOClient.get_instance().emit(STATIC_VAR.START_STREAM_API, start_stream)
        self.last_text = robot_id

    def run(self):
        while True:
            # print("hear")
            try:
                REDIS = redis.Redis(host=self.HOST, port=int(self.PORT), db=0, password=self.PASS)
                robots = REDIS.get("robots")
                # print("robots : ", robots)
            except Exception as e:
                # print(e)
                time.sleep(10)
                continue
            if robots is None:
                self.comboBox.clear()
                time.sleep(10)
                continue
            data_robots = json.loads(robots)
            itemSelected = self.comboBox.currentText()
            self.comboBox.clear()
            for index, robot in enumerate(data_robots):
                self.comboBox.addItem(robot["id"])
                if itemSelected == robot["id"]:
                    self.comboBox.setCurrentIndex(index)
                
            time.sleep(10)

    
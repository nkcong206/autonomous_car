from PyQt5 import QtCore, QtGui, QtWidgets
from module.GmapGui import GoogleMap
from module.Stream import ShowCamera
from module.Common import *
from module.LoadRobot import LoadRoBot
from module.Static import STATIC_VAR
from connection.Ps4Controller import Ps4Controller
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QMainWindow
import platform
from PyQt5.QtWidgets import QMessageBox
from connection.SocketClient import SocketIOClient
import datetime


class AutoCarWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1124, 731)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.googleMap = GoogleMap().get_instance()
        self.googleMap.setGeometry(0, 0, 200, 200)
        self.googleMap.setParent(MainWindow)

        self.gr1 = QtWidgets.QGroupBox(self.centralwidget)
        self.gr1.setGeometry(QtCore.QRect(640, 20, 461, 541))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.gr1.setFont(font)
        self.gr1.setObjectName("gr1")
        self.comboBox = QtWidgets.QComboBox(self.gr1)
        self.comboBox.setGeometry(QtCore.QRect(20, 30, 211, 51))
        self.comboBox.setObjectName("comboBox")
        self.pushButton = QtWidgets.QPushButton(self.gr1)
        self.pushButton.setGeometry(QtCore.QRect(240, 40, 113, 32))
        self.pushButton.setObjectName("pushButton")
        self.imgStream = QtWidgets.QLabel(self.gr1)
        self.imgStream.setGeometry(QtCore.QRect(9, 80, 441, 401))
        self.imgStream.setStyleSheet("border: 1px solid black;")
        self.imgStream.setText("")
        self.imgStream.setObjectName("imgStream")
        self.label_2 = QtWidgets.QLabel(self.gr1)
        self.label_2.setGeometry(QtCore.QRect(20, 500, 431, 31))
        font = QtGui.QFont()
        font.setPointSize(17)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.gb2 = QtWidgets.QGroupBox(self.centralwidget)
        self.gb2.setGeometry(QtCore.QRect(640, 580, 461, 141))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.gb2.setFont(font)
        self.gb2.setObjectName("gb2")
        self.runManual = QtWidgets.QRadioButton(self.gb2)
        self.runManual.setGeometry(QtCore.QRect(100, 40, 100, 20))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.runManual.setFont(font)
        self.runManual.setObjectName("runManual")
        self.radioButton = QtWidgets.QRadioButton(self.gb2)
        self.radioButton.setGeometry(QtCore.QRect(200, 40, 100, 20))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.radioButton.setFont(font)
        self.radioButton.setObjectName("radioButton")
        self.label = QtWidgets.QLabel(self.gb2)
        self.label.setEnabled(False)
        self.label.setGeometry(QtCore.QRect(30, 40, 60, 21))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.goAll = QtWidgets.QPushButton(self.gb2)
        self.goAll.setGeometry(QtCore.QRect(40, 80, 151, 51))
        self.goAll.setObjectName("goAll")
        self.stopAll = QtWidgets.QPushButton(self.gb2)
        self.stopAll.setGeometry(QtCore.QRect(260, 80, 151, 51))
        self.stopAll.setObjectName("stopAll")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.platform = platform.system()

        self.camera = ShowCamera(self.imgStream)
        self.camera.start()
        self.robot = LoadRoBot(self.comboBox)
        self.robot.start()
        Ps4Controller.get_instance().contructor(self.label_2, self.comboBox)

        self.timer = QTimer(MainWindow)
        self.timer.timeout.connect(self.update_controller_status)
        self.timer.start(100)

        self.runManual.setChecked(True)
        self.pushButton.setEnabled(False)
        self.goAll.setEnabled(False)
        self.stopAll.setEnabled(False)

        self.runManual.clicked.connect(self.runManual_checked)
        self.radioButton.clicked.connect(self.runAutomatic_checked)

        # self.pushButton.clicked.connect(self.goStopRobot)
        self.goAll.clicked.connect(self.goAllRobot)
        self.stopAll.clicked.connect(self.stopAllRobot)
        
        self.googleMap.init(self.pushButton, self.comboBox)
        
        self.l_signal1 = QtWidgets.QLabel(self)
        self.l_signal1.setEnabled(False)
        self.l_signal1.setGeometry(QtCore.QRect(15, 610, 40, 110))
        self.l_signal1.setStyleSheet("border: 1px solid black;background-color: white;")
        self.l_signal2 = QtWidgets.QLabel(self)
        self.l_signal2.setEnabled(False)
        self.l_signal2.setGeometry(QtCore.QRect(56, 610, 40, 110))
        self.l_signal2.setStyleSheet("border: 1px solid black;background-color: white;")
        self.l_signal3 = QtWidgets.QLabel(self)
        self.l_signal3.setEnabled(False)
        self.l_signal3.setGeometry(QtCore.QRect(98, 610, 40, 110))
        self.l_signal3.setStyleSheet("background-color: red;")
        
        self.l_signal_reader = QtWidgets.QLabel(self)
        # self.l_signal_reader.setEnabled(False)
        self.l_signal_reader.setGeometry(QtCore.QRect(150, 610, 470, 55))
        self.l_signal_reader.setStyleSheet("background-color: white; color : black;")
        
        self.l_signal_sender = QtWidgets.QLabel(self.centralwidget)
        # self.l_signal_sender.setEnabled(False)
        self.l_signal_sender.setGeometry(QtCore.QRect(150, 667, 470, 55))
        self.l_signal_sender.setStyleSheet("background-color: white; color : black;")
        # self.l_signal_sender.setText("aaaaaaaaaaaa")
        
        self.timer_sender = QTimer(MainWindow)
        self.timer_sender.timeout.connect(self.send_signal_2_robot)
        self.timer_sender.start(20000)
        SocketIOClient.get_instance().contructor(self.l_signal_reader)
        
    def send_signal_2_robot(self):
        mess = "255,0,0,12323313131"
        time = datetime.datetime.now()
        new_line = f"{time} {mess}\n"
        data_send = new_line
        text = self.l_signal_sender.text()
        robot_id = self.comboBox.currentText()
        if not len(text):
            self.l_signal_sender.setText(new_line)
        else:
            lines = text.split("\n")
            for index, line in enumerate(lines):
                if index >= 2:
                    break
                if index == 1:
                    new_line += f"{line}"
                if index == 0:
                    new_line += f"{line}\n"
            self.l_signal_sender.setText(new_line)
        data = {"robot_id" : robot_id, "data" : data_send}
        SocketIOClient.get_instance().emit("send_signal", data)
        

    def goAllRobot(self):
        status = SocketIOClient.get_instance().emit("automatic_all_robot", {"type" : STATIC_VAR.GO})
        if not status:
            QMessageBox.about(self, "ERROR", "Cant connect to server !!!")
        # else:
        #     self.pushButton.setText(STATIC_VAR.STOP)
        
    def stopAllRobot(self):
        status = SocketIOClient.get_instance().emit("automatic_all_robot", {"type" : STATIC_VAR.STOP})
        
        if not status:
            QMessageBox.about(self, "ERROR", "Cant connect to server !!!")
        # else:
        #     self.pushButton.setText(STATIC_VAR.GO)
            
    # def goStopRobot(self):
    #     robot_id = self.comboBox.currentText()
    #     if not len(robot_id):
    #         QMessageBox.about(self, "ERROR", "Please, select the robot that you want to control !!!")
    #         return

    #     if self.pushButton.text() == STATIC_VAR.GO:
    #         self.pushButton.setText(STATIC_VAR.STOP)
    #     else:
    #         self.pushButton.setText(STATIC_VAR.GO)

    #     status = SocketIOClient.get_instance().emit("run_automatic", {'robot_id' : robot_id, 'type' : self.pushButton.text()})
    #     if not status:
    #         QMessageBox.about(self, "ERROR", "Cant connect to server !!!")

    def runAutomatic_checked(self):
        self.pushButton.setEnabled(True)
        self.goAll.setEnabled(True)
        self.stopAll.setEnabled(True)
        STATIC_VAR.IS_AUTOMATIC = True

    def runManual_checked(self):
        self.pushButton.setEnabled(False)
        self.goAll.setEnabled(False)
        self.stopAll.setEnabled(False)
        STATIC_VAR.IS_AUTOMATIC = False

    def update_controller_status(self):
        if self.platform == STATIC_VAR.LINUX:
            Ps4Controller.get_instance().run_linux()
        else:
            Ps4Controller.get_instance().run()

    def keyPressEvent(self, event):
        key_text = event.text()
        Ps4Controller.get_instance().check_key_press(key_text)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.gr1.setTitle(_translate("MainWindow", "Selective Rebot"))
        self.pushButton.setText(_translate("MainWindow", "Go"))
        self.label_2.setText(_translate("MainWindow", "In manual mode, drive this rebot by a PS4 controller"))
        self.gb2.setTitle(_translate("MainWindow", "All cars"))
        self.runManual.setText(_translate("MainWindow", "Manual"))
        self.radioButton.setText(_translate("MainWindow", "Automatic"))
        self.label.setText(_translate("MainWindow", "Mode"))
        self.goAll.setText(_translate("MainWindow", "Go all"))
        self.stopAll.setText(_translate("MainWindow", "Stop all"))


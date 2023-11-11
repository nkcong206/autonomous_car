from PyQt5 import QtCore, QtGui, QtWidgets
from module.Gui import AutoCarWindow
from connection.SocketClient import SocketIOClient

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = AutoCarWindow()
    MainWindow.show()

    SocketIOClient.get_instance().start()
    
    sys.exit(app.exec_())

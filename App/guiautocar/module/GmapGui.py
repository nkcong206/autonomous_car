import sys
import json
import io
import folium
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView
from module.Draw import Draw
from PyQt5 import QtWebEngineWidgets
import geocoder
from bs4 import BeautifulSoup
from PyQt5.QtCore import QTimer
import threading
from module.Static import STATIC_VAR
from PyQt5.QtWidgets import QMessageBox
from module.GmapDirectionApi import GMapDirectionApi
from connection.SocketClient import SocketIOClient

class GoogleMap(QWidget):
    instance = None
    @staticmethod
    def get_instance():
        if GoogleMap.instance is None:
            GoogleMap.instance = GoogleMap()
        return GoogleMap.instance

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Folium")
        self.window_width, self.window_height = 640, 600
        self.setMinimumSize(self.window_width, self.window_height)

        self.list_robot_ids = {}
        self.list_position_robot = []

        class WebEnginePage(QtWebEngineWidgets.QWebEnginePage):
            def javaScriptConsoleMessage(self, level, msg, line, sourceID):
                try:
                    coords_dict = json.loads(msg)
                    if coords_dict["type"] == STATIC_VAR.DELETE:
                        lat = coords_dict["lat"]
                        lng = coords_dict["lng"]
                        for index, (lat_, lng_) in enumerate(STATIC_VAR.LIST_CHECKPOINT):
                            if lat == lat_ and lng == lng_:
                                del STATIC_VAR.LIST_CHECKPOINT[index]
                    elif coords_dict["type"] == STATIC_VAR.CREATE:
                        lng, lat = coords_dict["geometry"]["coordinates"]
                        STATIC_VAR.LIST_CHECKPOINT.append([lat, lng])
                except:
                    pass

        layout = QVBoxLayout()
        self.setLayout(layout)

        current_location = self.get_current_location()

        self.lock = threading.Lock()

        m = folium.Map(location=current_location, zoom_start=20)

        draw = Draw(
            draw_options={
                'polyline':False,
                'rectangle':False,
                'polygon':False,
                'circle':False,
                'marker':True,
                'circlemarker':False},
            edit_options={'edit': False})
        
        m.add_child(draw)

        data = io.BytesIO()
        m.save(data, close_file=False)
        
        m.save("index.html")

        self.idRoot = self.get_root_id(data.getvalue().decode())
        
        self.webView = QWebEngineView()
        page = WebEnginePage(self.webView)
        self.webView.setPage(page)
        self.webView.setHtml(data.getvalue().decode())
        layout.addWidget(self.webView)

        self.updateLocationRobotTimer = QTimer()
        self.updateLocationRobotTimer.timeout.connect(self.draw_position)
        self.updateLocationRobotTimer.start(100)
        
    def init(self, buttonGoStop, comboxRobot):
        self.buttonGoStop = buttonGoStop
        self.comboxRobot = comboxRobot
        self.buttonGoStop.clicked.connect(self.goStopRobot)
        
    def goStopRobot(self):
        robot_id = self.comboxRobot.currentText()
        if not len(robot_id):
            QMessageBox.about(self, "ERROR", "Please, select the robot that you want to control !!!")
            return
        textButton = self.buttonGoStop.text()
        if textButton == STATIC_VAR.GO:
            locations = STATIC_VAR.LIST_CHECKPOINT
            if len(locations) >= 2:
                all_locations = GMapDirectionApi.get_instance()(locations)
                SocketIOClient.get_instance().emit("locations_direction", {"locations" : all_locations, "robot_id" : robot_id})
                self.draw_line(all_locations)
                STATIC_VAR.LIST_CHECKPOINT = []
        if textButton == STATIC_VAR.GO:
            self.buttonGoStop.setText(STATIC_VAR.STOP)
        else:
            self.buttonGoStop.setText(STATIC_VAR.GO)

        status = SocketIOClient.get_instance().emit("run_automatic", {'robot_id' : robot_id, 'type' : textButton})
        if not status:
            QMessageBox.about(self, "ERROR", "Cant connect to server !!!")
        
    def create_new_object(self, robot_id, location):
        lat, lng = location
        key_robot = "marker_" + str(robot_id)
        js_code = """
                var """ + key_robot + """ = L.marker(
                    [""" + str(lat) + """, """ + str(lng) + """], {}
                ).addTo(""" + self.idRoot + """);
            
                var icon_793f301fb50fb97cf5b13e158fbe3b59 = L.AwesomeMarkers.icon(
                    {"extraClasses": "fa-rotate-0", "icon": "robot", "iconColor": "white", "markerColor": "red", "prefix": "fa"}
                );
                """ + key_robot + """.setIcon(icon_793f301fb50fb97cf5b13e158fbe3b59);
                """ + key_robot + """.bindTooltip('""" + robot_id + """', {
                    permanent: true,
                    direction: 'right',
                    offset: [10, -25],
                    noWrap: true
                });
            """
        self.list_robot_ids[robot_id] = key_robot
        self.webView.page().runJavaScript(js_code)
        
    def draw_line(self, locations):
        js_code = """
            var poly_line_97eb3a2b7be452b7f23b246ade3b9243 = L.polyline(
                """ + str(locations) + """,
                {"bubblingMouseEvents": true, "color": "red", "dashArray": null, "dashOffset": null, "fill": false, "fillColor": "red", "fillOpacity": 0.2, "fillRule": "evenodd", "lineCap": "round", "lineJoin": "round", "noClip": false, "opacity": 1, "smoothFactor": 1.0, "stroke": true, "weight": 4}
            ).addTo(""" + self.idRoot + """);
        """
        self.webView.page().runJavaScript(js_code)
        

    def update_location(self, robot_id, location):
        lat, lng = location
        key_robot = self.list_robot_ids[robot_id]
        js_code = key_robot + """.setLatLng([""" + str(lat) + """,""" + str(lng) + """])"""
        self.webView.page().runJavaScript(js_code)

    def remove_robot_on_map(self, robot_id):
        self.lock.acquire()
        key_robot = self.list_robot_ids[robot_id]
        js_code = key_robot + ".remove()"
        self.webView.page().runJavaScript(js_code)
        self.lock.release()
        
    def get_location_robot_by_id(self, robot_id):
        lat, lng = None, None
        flag = 0
        for robot_location in self.list_position_robot:
            if robot_location["robot_id"] == robot_id:
                lat, lng = robot_location["location"]
                flag = 1
        return flag, (lat, lng)

    def draw_position(self):
        if not len(self.list_position_robot):
            return
        item = self.list_position_robot[0]
        if item['robot_id'] not in self.list_robot_ids.keys():
            self.create_new_object(item["robot_id"], item["location"])
        else:
            self.update_location(item["robot_id"], item["location"])
        del self.list_position_robot[0]

    def update_location_robot(self, robot_id, location):
        self.lock.acquire()
        self.list_position_robot.append({'robot_id': robot_id, 'location' : location})
        self.lock.release()
    
    def get_root_id(self, html):
        soup = BeautifulSoup(html,features="html.parser")
        div = soup.find('div', attrs={'class' : 'folium-map'})
        id = div.get_attribute_list("id")[0]
        return id

    def get_current_location(self):    
        location = geocoder.ip('me')
        return location.latlng

from PyQt5 import QtCore, QtWidgets, QtWebEngineWidgets
# from folium.plugins import Draw
from MarkerEdit import MarkerEdit
from Draw import Draw
import time
import folium, io, sys, json
from PyQt5.QtWebEngineWidgets import QWebEnginePage
import threading
   

if __name__ == '__main__': 
    app = QtWidgets.QApplication(sys.argv)
    
    m = folium.Map(location=[55.8527, 37.5689], zoom_start=13)
    
    draw = Draw(
        draw_options={
            'polyline':False,
            'rectangle':False,
            'polygon':False,
            'circle':False,
            'marker':True,
            'circlemarker':False},
        edit_options={'edit': True})
    m.add_child(draw)

    # folium.Marker(
    #     location=[55.8527, 37.5689],
    #     tooltip="Click me!",
    #     popup="Mt. Hood Meadows",
    #     icon=folium.Icon(icon="cloud"),
    # ).add_to(m)

    point = MarkerEdit(
        location=[55.8527, 37.5689],
        tooltip=folium.Tooltip(text="<strong>test</strong>", permanent=True, direction='center'),
        popup="Mt. Hood Meadows",
        icon=folium.Icon(icon="cloud"),
        id = "Trung"
    )

    point.add_to(m)

    for key in m._children:
      print("================")
      print(key)
      print("================")
      if key.startswith('marker'):
        print("key : ", m._children[key].id, key)
    
    # m.save("index.html")
    data = io.BytesIO()
    m.save(data, close_file=False)

    class WebEnginePage(QtWebEngineWidgets.QWebEnginePage):
       def javaScriptConsoleMessage(self, level, msg, line, sourceID):
          print("===========")
          print(msg)
          print("===========")
          # Thay đổi toạ độ của Marker
        #   new_latitude = 40.7128
        #   new_longitude = -74.0060

        #   new_marker = folium.Marker([new_latitude, new_longitude], popup='New Marker')

        #   new_marker.add_to(m)

    js_code = """
        console.log('hello world');
        var marker_50bdcc8bf14b37be865d782f21e51e52 = L.marker(
                [55.8927, 37.5789],
                {}
            ).addTo(map_d20d9f3cd6cc09863a610f3999405664);
        
    
            var icon_793f301fb50fb97cf5b13e158fbe3b59 = L.AwesomeMarkers.icon(
                {"extraClasses": "fa-rotate-0", "icon": "cloud", "iconColor": "white", "markerColor": "blue", "prefix": "glyphicon"}
            );
            marker_50bdcc8bf14b37be865d782f21e51e52.setIcon(icon_793f301fb50fb97cf5b13e158fbe3b59);
    """

    # print("page : ", data.getvalue().decode())
    view = QtWebEngineWidgets.QWebEngineView()
    page = WebEnginePage(view)
    view.setPage(page)
    view.setHtml(data.getvalue().decode())

    # view.page().runJavaScript(js_code)
    # t1 = threading.Thread(target=RunStart, args=(view))
    # t1.start()

    view.show()
    sys.exit(app.exec_())
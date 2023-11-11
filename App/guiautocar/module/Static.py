from dotenv import load_dotenv
import os

load_dotenv()

class STATIC_STREAM:
    Resolution = (441, 401)
    Url = "" #"rtmp://192.168.2.123:1935/live/test"
    Restart = False
    IsStream = True
    
class STATIC_VAR:
    SERVER_SOCKET = os.getenv('SERVER_SOCKET')
    SOCKET = None
    STOP_STREAM_API = "stop_stream"
    START_STREAM_API = "start_stream"
    LINUX = 'Linux'
    MACOS = "Drawin"
    WINDOWN = "Windows"
    LIST_CHECKPOINT = []
    DELETE = "DELETE"
    CREATE = "CREATE"
    CURRENT_LOCATION_ROBOT = None
    GO = "Go"
    STOP = "Stop"
    IS_AUTOMATIC = False
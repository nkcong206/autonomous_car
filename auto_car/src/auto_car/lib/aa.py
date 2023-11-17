import os
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

class aaaaaa():
    def paaa():
        print("asdasd")
        
    def load_env_from_config(package_name, config_folder='config'):
        package_share_directory = get_package_share_directory(package_name)
        dotenv_path = os.path.join(package_share_directory, config_folder, '.env')

        if os.path.exists(dotenv_path):
            load_dotenv(dotenv_path)
            SERVER_SOCKETIO = os.getenv("SERVER_SOCKETIO")
            print(SERVER_SOCKETIO)
        else:
            print(f"Error: {dotenv_path} not found. Make sure the file exists.")

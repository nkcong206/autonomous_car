import rospkg

def get_package_directory(package_name):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package_name)
    return package_path

# Thay thế 'your_package_name' bằng tên của package chính bạn muốn tìm đường dẫn
package_name = 'auto_car'
package_directory = get_package_directory(package_name)

if package_directory:
    print(f"Đường dẫn tới thư mục của package {package_name} là: {package_directory}")
else:
    print(f"Không tìm thấy package {package_name} trong ROS.")
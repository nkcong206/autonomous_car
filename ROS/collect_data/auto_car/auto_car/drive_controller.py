import rclpy
import math
# from pyPS4Controller.controller import Controller
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from pop import Pilot

<<<<<<< HEAD
class DriveController(Node):
    wheel_base = 0.12   # 12 cm, lấy từ file URDF. Cần đo lại?

    # tham số mô hình động học
    a_bm = 0.015           # theo thông số nsx, tốc độ tối đa là 1.5 m/s, lấy 99/1.5 = 66
    # a_bm = 0.66         # nếu động cơ xuất từ 0 tới 0.99 thì a_bm = 0.66
    b_bm = 0.0            # chỗ này có thể cần xem lại
    a_fm = math.pi/4    # nếu bị bẻ lái ngược thì sửa lại thành -pi/4
    b_fm = 0.0            # nếu động cơ lái bị lệch thì mới != 0
=======

class DriveController(Node):
    # khoảng cách giữa trục bánh trước và trục bánh sau
    # wheel_base = 260    # thông số độ dài cần theo đơn vị mét
    wheel_base = 0.12   # 12 cm, lấy từ file URDF. Cần đo lại?

    # tham số mô hình động học
    a_bm = 1           # theo thông số nsx, tốc độ tối đa là 1.5 m/s, lấy 99/1.5 = 66
    # a_bm = 0.66         # nếu động cơ xuất từ 0 tới 0.99 thì a_bm = 0.66
    b_bm = 0            # chỗ này có thể cần xem lại
    a_fm = 1    # nếu bị bẻ lái ngược thì sửa lại thành -pi/4
    b_fm = 0            # nếu động cơ lái bị lệch thì mới != 0
>>>>>>> origin/main

    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
<<<<<<< HEAD
=======

>>>>>>> origin/main
        # Dữ liệu sử dụng lại giữa các lần lấy mẫu
        self.msg_odom = Odometry()
        self.yaw = 0.0
        self.msg_joint_states = JointState()
        self.last_time = self.get_clock().now()

        # set giá trị ban đầu cho msg_odom
        self.msg_odom.header.frame_id = "odom"          # hệ quy chiếu gốc
        self.msg_odom.child_frame_id = "base_link"      # hệ quy chiếu đích
        self.msg_odom.pose.pose.position.x = 0.0        # vị trí - x
        self.msg_odom.pose.pose.position.y = 0.0        # vị trí - y
<<<<<<< HEAD
        self.msg_odom.pose.pose.position.z = 0.0      # vị trí - z      0.032 
        self.msg_odom.pose.pose.orientation.x = 0.0     # quaternion phép xoay hệ quy chiếu - x
        self.msg_odom.pose.pose.orientation.y = 0.0     # quaternion phép xoay hệ quy chiếu - y
        self.msg_odom.pose.pose.orientation.z = 0.0     # quaternion phép xoay hệ quy chiếu - z
        self.msg_odom.pose.pose.orientation.w = 0.0     # quaternion phép xoay hệ quy chiếu - w     1.0
=======
        self.msg_odom.pose.pose.position.z = 0.032      # vị trí - z
        self.msg_odom.pose.pose.orientation.x = 0.0     # quaternion phép xoay hệ quy chiếu - x
        self.msg_odom.pose.pose.orientation.y = 0.0     # quaternion phép xoay hệ quy chiếu - y
        self.msg_odom.pose.pose.orientation.z = 0.0     # quaternion phép xoay hệ quy chiếu - z
        self.msg_odom.pose.pose.orientation.w = 1.0     # quaternion phép xoay hệ quy chiếu - w
>>>>>>> origin/main
        self.msg_odom.twist.twist.linear.x = 0.0        # tốc độ tịnh tiến (được quyết định bởi động cơ sau)
        self.msg_odom.twist.twist.linear.y = 0.0        # tốc độ tịnh tiến theo phương ngang hông xe (odometry xe không đo được)
        self.msg_odom.twist.twist.linear.z = 0.0        # tốc độ tịnh tiến theo phương thẳng đứng (odometry xe không đo được)
        self.msg_odom.twist.twist.angular.z = 0.0       # tốc độ xoay theo trục x (odometry xe không đo được)
        self.msg_odom.twist.twist.angular.z = 0.0       # tốc độ xoay theo trục y (odometry xe không đo được)
        self.msg_odom.twist.twist.angular.z = 0.0       # tốc độ xoay vòng của xe
        self.msg_odom.pose.covariance[0] = 0.2          # phương sai vị trí - x
        self.msg_odom.pose.covariance[7] = 0.2          # phương sai vị trí - y
        self.msg_odom.pose.covariance[35] = 0.4         # phương sai góc hướng đầu

        # set gíá trị ban đầu cho joint_state
        self.msg_joint_states.header.frame_id = "base_link"
        self.msg_joint_states.name = [
            'steering_joint1',
            'wheel_joint1',
            'steering_joint2',
            'wheel_joint2',
            'wheel_joint3',
            'wheel_joint4',
            'pan',
            'tilt',
        ]
        self.msg_joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Tham chiếu API xe
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(0)

        # publisher và subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.joint_states_pub = self.create_publisher(JointState, "/joint_states", 10)

        # transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Tạo timer định kỳ xuất odometry, tần số f_pub = 40 Hz
        timer_period = 0.025    # chu kỳ dt = 1/f_pub = 0.025 s
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_vel_callback(self, cmd_vel_msg: Twist):
        
        # if cmd_vel_msg.linear.x == 0:
        #     self.msg_odom.twist.twist.linear.x = 0.0
        #     self.msg_odom.twist.twist.angular.z = 0.0
        #     self.car.stop()
        #     return

        # # tính steering_angle
        # steering_angle = math.atan2(
        #     cmd_vel_msg.angular.z*self.wheel_base,
        #     cmd_vel_msg.linear.x)

        # tính tín hiệu các động cơ (công thức hồi quy chuyển vế ngược lại)
        front_motor = cmd_vel_msg.angular.z
        back_motor = cmd_vel_msg.linear.x

        # kiểm tra giới hạn
        if back_motor < -99: back_motor = -99
        if back_motor > 99: back_motor = 99
        if front_motor < -1.0: front_motor = -1.0
        if front_motor > 1.0: front_motor = 1.0


        # set tốc độ động cơ
        if back_motor > 0:
            self.car.forward(back_motor)
        elif back_motor < 0:
            self.car.backward(-back_motor)
        else:
            self.car.stop()

        # set servo lái
        #self.car.steering = front_motor
        self.get_logger().info(F"{back_motor}, {front_motor}")
        self.car.steering = front_motor

        # tính ngược lại tốc độ (sau khi giới hạn mức động cơ)
        constrained_speed = self.a_bm*back_motor + self.b_bm
        constained_steering_angle = self.a_fm*front_motor + self.b_fm
        constrained_yaw_rate = constrained_speed * math.tan(constained_steering_angle) / self.wheel_base

        # lưu lại để publish odometry
        self.msg_odom.twist.twist.linear.x = constrained_speed
        self.msg_odom.twist.twist.angular.z = constrained_yaw_rate

        # lưu lại trạng thái khớp lái
        self.msg_joint_states.position[0] = constained_steering_angle
        self.msg_joint_states.position[2] = constained_steering_angle

    def timer_callback(self):

        # Note: Khi nào đọc được hồi tiếp, hoặc ngoại suy được 2 biến này từ
        # mô hình khác thì mới cần cập nhật lại:
        # speed = self.a_bm*back_motor + self.b_bm
        # steering_angle = self.a_fm*front_motor + self.b_fm
        # angular_velocity = self.speed*math.tan(self.steering_angle)/ self.wheel_base

        # xử lý thời gian
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        self.msg_odom.header.stamp = current_time.to_msg()

        # tốc độ đã được lưu trong msg_odom:
        speed = self.msg_odom.twist.twist.linear.x
        angular_vel = self.msg_odom.twist.twist.angular.z

        # vector tốc độ trong hệ quy chiếu "odom" (xoay 1 góc = self.yaw)
        x_dot = speed*math.cos(self.yaw)
        y_dot = speed*math.sin(self.yaw)

        # lấy tích phân tốc độ --> vị trí
        self.msg_odom.pose.pose.position.x += x_dot*dt
        self.msg_odom.pose.pose.position.y += y_dot*dt

        # tích phân tốc độ góc --> góc hướng đầu xe
        self.yaw += angular_vel*dt

        # cập nhật quaternion
        self.msg_odom.pose.pose.orientation.z = math.sin(self.yaw/2.0)
        self.msg_odom.pose.pose.orientation.w = math.cos(self.yaw/2.0)

        # xuất bản msg
        self.odom_pub.publish(self.msg_odom)

        # xuất bản transform
        tf = TransformStamped()

        # set giá trị ban đầu cho transform
        tf.header.frame_id = self.msg_odom.header.frame_id      # hệ quy chiếu gốc
        tf.header.stamp = self.msg_odom.header.stamp
        tf.child_frame_id = self.msg_odom.child_frame_id        # hệ quy chiếu đích
        tf.transform.translation.x = self.msg_odom.pose.pose.position.x
        tf.transform.translation.y = self.msg_odom.pose.pose.position.y
        tf.transform.translation.z = self.msg_odom.pose.pose.position.z
        tf.transform.rotation.x = self.msg_odom.pose.pose.orientation.x
        tf.transform.rotation.y = self.msg_odom.pose.pose.orientation.y
        tf.transform.rotation.z = self.msg_odom.pose.pose.orientation.z
        tf.transform.rotation.w = self.msg_odom.pose.pose.orientation.w

        # broadcast transform
        self.tf_broadcaster.sendTransform(tf)

        # xuất bản joint_state
        self.msg_joint_states.header.stamp = self.msg_odom.header.stamp
        self.joint_states_pub.publish(self.msg_joint_states)

def main(args=None):
    rclpy.init(args=args)
    node = DriveController()
    try:    # try - except ở đây để khi dừng bằng Ctrl+C không báo lỗi
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

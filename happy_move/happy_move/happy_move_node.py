import math
import sys
import rclpy
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry    # Odometryメッセージ型をインポート
from tf_transformations import euler_from_quaternion 
from rclpy.duration import Duration


class HappyMove(Node):  # 簡単な移動クラス
    def __init__(self):   # コンストラクタ
        super().__init__('happy_move_node')        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.liner, self.angular = 0.0, 0.0
        self.vel = Twist()  # Twist メッセージ型インスタンスの生成
        self.set_vel(0.0, 0.0)  # 速度の初期化
 
    def get_pose(self, msg):      # 姿勢を取得する
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw

    def get_twist(self, msg):
        liner = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z

        return liner, angular
  
    def odom_cb(self, msg):         # オドメトリのコールバック関数
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.get_logger().info(
            f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad/s]')    

        self.liner, self.angular = self.get_twist(msg)
    
    def set_vel(self, linear, angular):  # 速度を設定する
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    
    
    def move_distance(self, dist):  # 指定した距離distを移動する
        error = 0.05  # 許容誤差 [m] 
        diff = dist - math.sqrt((self.x-self.x0)**2 + (self.y-self.y0)**2) 
        if math.fabs(diff) > error:
            self.set_vel(0.25, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def rotate_angle(self, angle):  # 指定した角度angleを回転する
        error = 0.01

        # 目標の角度を0からpiの値で設定
        target_angle = self.yaw0 + math.pi + angle 
        now_angle = self.yaw + math.pi

        if target_angle >= (math.pi * 2):
            target_angle = target_angle % (math.pi * 2)

        if now_angle >= (math.pi * 2):
            now_angle = now_angle % (math.pi * 2)

        diff = target_angle - now_angle
        if abs(diff) > error:
            self.set_vel(0.0, 0.1)
            return False
        else:
            self.set_vel(0.0, 0.0)
            rclpy.spin_once(self)
            return True

    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ 

    def move_time(self, time):
        now_time = self.get_clock().now()
        diff = now_time - self.start_time
        if diff > time:
            self.set_vel(0.0, 0.0)
            rclpy.spin_once(self)
            return True
        return False
            
    def draw_square(self, x):
        count = 0
        while rclpy.ok():
            if self.move_distance(x):
                if self.rotate_angle(math.pi/2):
                    self.x0 = self.x
                    self.y0 = self.y
                    self.yaw0 = self.yaw
                    count +=1
        
            if count == 4:
                rclpy.spin_once(self)
                break
            rclpy.spin_once(self)

    # 停止ができない
    def draw_circle(self, r):
        vel = 0.25
        angler = vel / r
        error = 0.01
        count = 0

        while rclpy.ok():
            self.show_tire_radius()
            if count == 0:
                count = 1
                self.set_vel(vel, angler)
                rclpy.spin_once(self)
            elif count == 2:
                rclpy.spin_once(self)
                break
            
            if self.yaw == 0 & count == 1:
                self.set_vel(0.0, 0.0)
                count = 2
                rclpy.spin_once(self)
                
            rclpy.spin_once(self)

    def go_to_odom(self, x, y):
        angle_radius = math.atan2(y, x)
        dist = math.sqrt((x**2 + y**2))
        error = 0.01
        
        while rclpy.ok():
            if self.rotate_angle(angle_radius):
                if self.move_distance(dist):
                    rclpy.spin_once(self)
                    break
            rclpy.spin_once(self)

    def show_tire_radius(self):
        Vel = self.liner
        Omega = self.angular

        d = 143.5 / 100
        r = 70 / 100

        omega_left = (Vel - Omega * d) / r
        omega_right = (Vel + Omega * d) / r

        self.get_logger().info(
            f'omega_left={omega_left: .2f}[rad/s] omega_right={omega_right: .2f}[rad/s]') 

    def happy_move(self, distance, angle, time, linear, angular, r, x, y):  # 簡単な状態遷移
        state = 3
        self.start_time = self.get_clock().now()

        if state == 0:
            while rclpy.ok():
                if state == 0:
                    if self.move_distance(distance):
                        state = 1
                elif state == 1:                
                    if self.rotate_angle(angle):
                        break
                else:
                    print('エラー状態')
                rclpy.spin_once(self)

        elif state == 1:
            self.set_vel(linear, angular)
            self.duration_time = Duration(seconds = time)
            while rclpy.ok():
                if self.move_time(self.duration_time):
                    break
                rclpy.spin_once(self)
        elif state == 2:
            self.draw_square(distance)
        elif state == 3:
            self.draw_circle(r)
        else:
            self.go_to_odom(x, y)


def main(args=None):  # main関数
    rclpy.init(args=args)
    node = HappyMove()

    try:
        node.happy_move(2.0, math.pi/2, 10.0, 1.0, math.pi/3, 2.0, 3.0, 2.0)
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')     
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()

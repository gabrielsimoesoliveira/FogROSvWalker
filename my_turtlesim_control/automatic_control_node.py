# ~/fog_ws/src/my_turtlesim_control/my_turtlesim_control/automatic_control_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32

from math import cos, pi, sin, tanh, sqrt, fabs
import time
import numpy as np

class TurtleControlNode(Node):
    def __init__(self):
        super().__init__('turtle_control_node')
        self.get_logger().info("Iniciando controle de trajetória para a tartaruga...")
        
        self.init_parameters()
        self.init_variables()
        
        self.pose_sub = self.create_subscription(
            Pose,
            self.pose_topic,
            self.pose_callback,
            10)
        
        self.twist_pub = self.create_publisher(Twist, self.twist_topic, 5)
        
        # O timer é criado, mas a lógica só inicia quando a primeira pose for recebida
        self.timer = self.create_timer(1.0 / self.control_rate, self.timer_callback)

    def init_parameters(self):
        self.debug = True

        self.pose_topic = "/turtle1/pose"
        self.twist_topic = "/turtle1/cmd_vel"
        self.control_rate = 100.0
        self.goal_tolerances = {"xy": 0.3, "theta": 0.45}
        
        self.period = 20.0
        
        self.last_trajectory_x, self.last_trajectory_y = -0.1, -0.1
        self.started = False
        
        self.i_x, self.i_y = 1.0, 1.0
        self.k_x, self.k_y = 0.5, 0.5
        self.l_x, self.l_y = 1.0, 1.0
        self.a = 0.2
        
        self.start_x = None  # Ponto inicial agora será capturado dinamicamente
        self.start_y = None
        self.stop_tolerance = 0.2
        self.min_time_to_check = self.period * 0.75

    def init_variables(self):
        self.current_pose = Pose()
        self.pose_count = 0
        self.start_time = None
        self.time_offset = self.period / 4.0
        
        self.total_tracking_error = 0.0
        self.total_direction_change = 0.0
        self.last_ang_vel_ref = 0.0
        
    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_count += 1
        
        # --- A GRANDE MUDANÇA: Captura da posição inicial ---
        if self.start_x is None:
            self.start_x = msg.x
            self.start_y = msg.y
            self.start_time = time.time()
            self.get_logger().info(f'Posição inicial capturada: x={self.start_x:.2f}, y={self.start_y:.2f}')
        # ----------------------------------------------------

    def get_current_goal(self, t):
        self.trajectory_x = self.start_x + (-2 * cos(2 * pi * t / self.period) / (sin(2 * pi * t / self.period)**2 + 1))
        self.trajectory_y = self.start_y + (-2 * sin(2 * pi * t / self.period) * cos(2 * pi * t / self.period) / (sin(2 * pi * t / self.period)**2 + 1))

    def calculate_position_error(self):
        self.error_x = self.trajectory_x - self.current_pose.x
        self.error_y = self.trajectory_y - self.current_pose.y
        return

    def controller(self):
        current_time = time.time() - self.start_time
        
        if not self.started:
            self.last_time = -0.1
            self.started = True
            
        # Lógica de parada
        if current_time >= self.period:
            distance_to_start = sqrt((self.current_pose.x - self.start_x)**2 + (self.current_pose.y - self.start_y)**2)
            if distance_to_start < self.stop_tolerance:
                self.twist_pub.publish(Twist())
                
                self.get_logger().info(f'Trajeto de demiscata finalizado.')
                self.get_logger().info(f'Tempo total para o trajeto: {current_time:.2f} segundos.')
                self.get_logger().info(f'Erro de rastreamento total: {self.total_tracking_error:.2f}')
                self.get_logger().info(f'Soma das mudanças de direção: {self.total_direction_change:.2f} rad/s')
                
                self.timer.cancel()
                rclpy.shutdown()
                return

        t = current_time + self.time_offset
        self.get_current_goal(t)
        self.calculate_position_error()
        
        distance_to_goal = sqrt(self.error_x**2 + self.error_y**2)
        self.total_tracking_error += distance_to_goal
        
        delta_time = current_time - self.last_time
        if delta_time <= 0:
            delta_time = 0.001
            
        delta_trajectory_x = (self.trajectory_x - self.last_trajectory_x) / delta_time
        delta_trajectory_y = (self.trajectory_y - self.last_trajectory_y) / delta_time
        
        control_law_x = delta_trajectory_x + self.i_x * tanh(self.error_x * (self.k_x / self.l_x))
        control_law_y = delta_trajectory_y + self.i_y * tanh(self.error_y * (self.k_y / self.l_y))
        
        self.lin_vel_ref = cos(self.current_pose.theta) * control_law_x + sin(self.current_pose.theta) * control_law_y
        self.ang_vel_ref = (-1/self.a) * sin(self.current_pose.theta) * control_law_x + (1/self.a) * cos(self.current_pose.theta) * control_law_y
        
        self.total_direction_change += fabs(self.ang_vel_ref - self.last_ang_vel_ref)
        self.last_ang_vel_ref = self.ang_vel_ref
        
        self.last_trajectory_x, self.last_trajectory_y = self.trajectory_x, self.trajectory_y
        self.last_time = current_time
        
        msg = Twist()
        msg.linear.x = self.lin_vel_ref
        msg.angular.z = self.ang_vel_ref
        self.twist_pub.publish(msg)

    def timer_callback(self):
        if self.start_x is not None:
            self.controller()
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

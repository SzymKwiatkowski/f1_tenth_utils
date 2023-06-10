import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from geometry_msgs.msg import PoseStamped

import math
import numpy as np
import pandas as pd

# Parameters
Kp = 1.0  # speed proportional gain
dt = 0.04  # [s] time tick
WB = 0.33  # [m] wheel base of vehicle

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, a=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
    
    def calc_distance(self, x, y):
        return math.sqrt((self.x-x)**2 + (self.y-y)**2)

class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

def yaw_from_quaternion(q):
    return np.arctan2(2.0*(q[0]*q[2] + q[3]*q[0]), q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2])

class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_f1_tenth_controller')
        
        options = QoSProfile(depth=1)
        options.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        options.reliability = QoSReliabilityPolicy.RELIABLE
        self.publisher_ = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', qos_profile=options)
        
        self.time_nanosecs = self.get_clock().now().nanoseconds
        self.state = State(x=0, y=0, yaw=yaw_from_quaternion([0.0,0.0,0.0,0.0]), v=0.0)
                
        self.target_speed = 1.0 # [units/s]
        
        self.pose_subscription_ = self.create_subscription(PoseStamped, '/ground_truth/pose', 
                                                           self.update_position, 10)
        
        self.pose_subscription_ = self.create_subscription(PoseStamped, '/goal_pose', 
                                                           self.control_to_goal, 10)
        
    def update_position(self, pose: PoseStamped):
        pose_l = [pose.pose.position.x, 
                 pose.pose.position.y,
                 pose.pose.position.z, 
                 pose.pose.orientation.x,
                 pose.pose.orientation.y, 
                 pose.pose.orientation.z, 
                 pose.pose.orientation.w]

        self.state = State(x=pose_l[0], y=pose_l[1], yaw=yaw_from_quaternion(pose_l[3:]), v=self.state.v)
        
    def proportional_control(self):
        a = Kp * (self.target_speed - self.state.v)

        return a

    def pure_pursuit_steer_control(self, target_state: State, Lf):
        
        if Lf == 0:
            return 0.0
    
        alpha = math.atan2(target_state.y - self.state.rear_y, target_state.x - self.state.rear_x) - self.state.yaw

        delta = math.atan2(2.0 * WB * math.sin(alpha), Lf)

        return delta
    
    def publish_control(self, steer, accel):
        acc_msg = AckermannControlCommand()
        acc_msg.lateral.steering_tire_angle = steer
        acc_msg.longitudinal.acceleration = accel
        acc_msg.longitudinal.speed = self.target_speed
        self.publisher_.publish(acc_msg)
        self.get_logger().info(f'Published acc: {accel} and steer: {steer}')
    
    def control_to_goal(self, pose: PoseStamped):
        target_state = State(x=pose.pose.position.x,
                             y=pose.pose.position.y,
                             yaw=yaw_from_quaternion([pose.pose.orientation.x,
                                                      pose.pose.orientation.y,
                                                      pose.pose.orientation.z,
                                                      pose.pose.orientation.w]))
        dst = target_state.calc_distance(self.state.x, self.state.y)
        ai = self.proportional_control()
        di = self.pure_pursuit_steer_control(self.state, dst)
        
        self.state.a = ai
        theta = self.state.yaw + di
        
        self.state.update(ai, theta)
        
        self.publish_control(di, ai)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PurePursuitController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
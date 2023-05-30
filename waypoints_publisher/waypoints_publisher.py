import rclpy
import json
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

class WaypointsPublisher(Node):

    def __init__(self):
        super().__init__('WaypointsPublisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.subsriber_ = self.create_subscription(PoseStamped, 'ground_truth/pose', 
                                                   self.pose_callback, 10)
        self.current_id = 0
        data = json.load(open('waypoints/waypoints.json'))
        waypoints = data['trajectory']
        self.poses = self.transform_to_pose(waypoints)
        self.poses_count = len(self.poses)
        self.error_allowed = 0.8

    def pose_callback(self, pose: PoseStamped):
        current_pose = self.poses[self.current_id]
        self.publisher_.publish(current_pose)
        
        if abs(current_pose.pose.position.x -pose.pose.position.x) < self.error_allowed and\
            abs(current_pose.pose.position.y -pose.pose.position.y) < self.error_allowed:
            self.current_id = (self.current_id+1) % self.poses_count
        
    def transform_to_pose(self, waypoints):
        poses = []
        scaler = 60
        for id in range(int(len(waypoints)/scaler)):
            pose = PoseStamped()
            pose.pose.position.x = waypoints[scaler*id][0]
            pose.pose.position.y = waypoints[scaler*id][1]
            poses.append(pose)
        return poses
        
            
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = WaypointsPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

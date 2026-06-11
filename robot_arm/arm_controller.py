import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacle_marker', 10)
        
        self.L1 = 0.5
        self.L2 = 0.4
        
        # Obstacle position and size
        self.obstacle = {'x': 0.4, 'y': 0.0, 'z': 0.4, 'radius': 0.1}
        
        # Targets to reach
        self.targets = [
            (0.5, 0.0, 0.3),
            (0.3, 0.2, 0.5),
            (0.4, -0.2, 0.2),
            (0.6, 0.0, 0.1),
        ]
        
        self.current_angles = [0.0, 0.0, 0.0]
        self.target_angles = [0.0, 0.0, 0.0]
        self.current_target = 0
        self.steps = 0
        self.total_steps = 50
        
        self.timer = self.create_timer(0.05, self.update)
        self.target_timer = self.create_timer(4.0, self.next_target)
        self.marker_timer = self.create_timer(0.5, self.publish_obstacle)
        
        self.get_logger().info('Arm Controller with Collision Avoidance Started!')
        self.next_target()

    def next_target(self):
        x, y, z = self.targets[self.current_target % len(self.targets)]
        
        # Check if target itself is inside obstacle
        if self.collides_with_obstacle(x, y, z):
            self.get_logger().warn(f'Target {self.current_target + 1} is inside obstacle! Skipping...')
            self.current_target += 1
            return
        
        angles = self.inverse_kinematics(x, y, z)
        if angles:
            # Check if path collides with obstacle
            if self.path_is_clear(self.current_angles, list(angles)):
                self.target_angles = list(angles)
                self.steps = 0
                self.get_logger().info(f'✅ Path clear! Moving to target {self.current_target + 1}: X={x}, Y={y}, Z={z}')
            else:
                self.get_logger().warn(f'⚠️  Collision detected on path to target {self.current_target + 1}! Finding safe path...')
                safe_angles = self.find_safe_path(x, y, z)
                if safe_angles:
                    self.target_angles = safe_angles
                    self.steps = 0
                    self.get_logger().info(f'✅ Safe path found!')
                else:
                    self.get_logger().warn('❌ No safe path found, skipping target.')
        self.current_target += 1

    def collides_with_obstacle(self, x, y, z):
        obs = self.obstacle
        dist = math.sqrt((x - obs['x'])**2 + (y - obs['y'])**2 + (z - obs['z'])**2)
        return dist < obs['radius']

    def path_is_clear(self, start_angles, end_angles, steps=20):
        for i in range(steps):
            t = i / steps
            mid_angles = [
                start_angles[j] + (end_angles[j] - start_angles[j]) * t
                for j in range(3)
            ]
            x, y, z = self.forward_kinematics(mid_angles)
            if self.collides_with_obstacle(x, y, z):
                return False
        return True

    def find_safe_path(self, x, y, z):
        # Try going higher to avoid obstacle
        waypoints = [
            (x, y, z + 0.2),
            (x + 0.1, y, z),
            (x - 0.1, y, z),
            (x, y + 0.1, z),
        ]
        for wx, wy, wz in waypoints:
            angles = self.inverse_kinematics(wx, wy, wz)
            if angles and self.path_is_clear(self.current_angles, list(angles)):
                self.get_logger().info(f'Going via waypoint: X={wx:.2f}, Y={wy:.2f}, Z={wz:.2f}')
                return list(angles)
        return None

    def forward_kinematics(self, angles):
        t1, t2, t3 = angles
        r = self.L1 * math.cos(t2) + self.L2 * math.cos(t2 + t3)
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        z = self.L1 * math.sin(t2) + self.L2 * math.sin(t2 + t3)
        return x, y, z

    def inverse_kinematics(self, x, y, z):
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        D = math.sqrt(r**2 + z**2)
        
        if D > (self.L1 + self.L2):
            return None
        
        cos_theta3 = (D**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        theta3 = math.acos(max(-1, min(1, cos_theta3)))
        alpha = math.atan2(z, r)
        beta = math.atan2(self.L2 * math.sin(theta3), self.L1 + self.L2 * math.cos(theta3))
        theta2 = alpha - beta
        
        return (theta1, theta2, theta3)

    def update(self):
        if self.steps < self.total_steps:
            t = self.steps / self.total_steps
            smooth_t = t * t * (3 - 2 * t)
            interpolated = [
                self.current_angles[i] + (self.target_angles[i] - self.current_angles[i]) * smooth_t
                for i in range(3)
            ]
            self.publish_joints(interpolated)
            self.steps += 1
        else:
            self.current_angles = list(self.target_angles)

    def publish_joints(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [angles[0], angles[1], angles[2]]
        self.joint_pub.publish(msg)

    def publish_obstacle(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.obstacle['x']
        marker.pose.position.y = self.obstacle['y']
        marker.pose.position.z = self.obstacle['z']
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.obstacle['radius'] * 2
        marker.scale.y = self.obstacle['radius'] * 2
        marker.scale.z = self.obstacle['radius'] * 2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
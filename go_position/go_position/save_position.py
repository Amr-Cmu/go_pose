import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import csv
import os
import sys

class RobotPoseLogger(Node):
    def __init__(self, path):
        super().__init__('save_position')
        
        base_path = '/home/thanawat/amr_ws/src/go_position/file_position'
        os.makedirs(base_path, exist_ok=True)
        self.path_csv = os.path.join(base_path, path)

        self.current_x = None
        self.current_y = None
        self.current_qx = None
        self.current_qy = None
        self.current_qz = None
        self.current_qw = None
        
        self.init_csv_file(self.path_csv)

        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.save_pose_subscriber = self.create_subscription(
            String,
            '/save_position',
            self.save_pose_callback,
            10
        )
        self.status_publisher = self.create_publisher(
            String,
            '/save_status',
            10
        )

    def init_csv_file(self, filename):

        if not os.path.exists(filename):
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Timestamp', 'X', 'Y', 'Orientation_X',
                            'Orientation_Y', 'Orientation_Z', 'Orientation_W',
                            'Name_Pose'])

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_qx = msg.pose.pose.orientation.x
        self.current_qy = msg.pose.pose.orientation.y
        self.current_qz = msg.pose.pose.orientation.z
        self.current_qw = msg.pose.pose.orientation.w
 
    def save_pose_callback(self, msg):
        data = msg.data.split(',')
        if len(data) != 2 or data[0].strip().lower() != 'true':
            self.publish_status('Invalid save_pose message format!')
            return
            
        pose_name = data[1].strip()
        
        existing_names = set()
        if os.path.exists(self.path_csv):
            with open(self.path_csv, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                existing_names = {row['Name_Pose'] for row in reader}
        
        if pose_name in existing_names:
            self.publish_status(f'Warning: Pose name "{pose_name}" already exists!')
            return
        
        if self.current_x is None:
            self.publish_status('No pose data available!')
            return
            
        try:
            timestamp = self.get_clock().now().to_msg().sec
            self.write_to_csv(self.path_csv, [
                timestamp,
                self.current_x,
                self.current_y,
                self.current_qx,
                self.current_qy,
                self.current_qz,
                self.current_qw,
                pose_name
            ])
            
            self.publish_status(f'Successfully saved pose: {pose_name}')
            
        except Exception as e:
            self.publish_status(f"Failed to save pose: {str(e)}")

    def write_to_csv(self, filename, data):
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)

    def publish_status(self, success):
        msg = String()
        msg.data = success
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    path = sys.argv[1]  
    node = RobotPoseLogger(path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
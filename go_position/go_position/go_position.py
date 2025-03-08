import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import os
import csv
import sys

class AMRNav(Node):
    def __init__(self, home):
        super().__init__('go_position')

        self.navigator = BasicNavigator()

        base_path = '/home/thanawat/amr_ws/src/go_position/file_position'
        self.csv_filename = os.path.join(base_path, home)

        self.landmarks = {}
        self.load_landmark_coordinates()

        self.status_publisher = self.create_publisher(
            String,
            '/go_position_status',
            10
        )
        self.nav_subscription = self.create_subscription(
            String,
            '/sub_position',
            self.navigate_callback,
            10
        )
        
        self.publish_status("ระบบนำทางพร้อมใช้งาน")

    def publish_status(self, status_msg):
        msg = String()
        msg.data = status_msg
        self.status_publisher.publish(msg)

    def load_landmark_coordinates(self):
        if not os.path.exists(self.csv_filename):
            error_msg = f'ไม่พบไฟล์ {self.csv_filename}!'
            self.publish_status(error_msg)
            return
            
        with open(self.csv_filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                try:
                    x, y, ox, oy, oz, ow = map(float, row[1:7])
                    pose_name = row[7]
                    self.landmarks[pose_name] = {
                        'x': x, 'y': y,
                        'ox': ox, 'oy': oy,
                        'oz': oz, 'ow': ow
                    }
                except (ValueError, IndexError) as e:
                    error_msg = f'ข้อมูลใน CSV ไฟล์ไม่ถูกต้อง: {row}'
                    self.publish_status(error_msg)

    def navigate_callback(self, msg):
        pose_name = msg.data.strip()
        
        if pose_name not in self.landmarks:
            error_msg = f'ไม่พบตำแหน่ง "{pose_name}" ในฐานข้อมูล'
            self.publish_status(error_msg)
            return
            
        self.publish_status(f'กำลังนำทางไปยังตำแหน่ง: {pose_name}')
        
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "map"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose.position.x = self.landmarks[pose_name]['x']
        pose_goal.pose.position.y = self.landmarks[pose_name]['y']
        pose_goal.pose.orientation.x = self.landmarks[pose_name]['ox']
        pose_goal.pose.orientation.y = self.landmarks[pose_name]['oy']
        pose_goal.pose.orientation.z = self.landmarks[pose_name]['oz']
        pose_goal.pose.orientation.w = self.landmarks[pose_name]['ow']
        
        self.navigator.goToPose(pose_goal)
  
        last_distance = None
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                current_distance = feedback.distance_remaining

                if last_distance is None or abs(current_distance - last_distance) > 0.5:
                    self.publish_status(
                        f'กำลังเดินทางไปยัง {pose_name}: เหลือระยะทาง {current_distance:.2f} เมตร'
                    )
                    last_distance = current_distance
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            success_msg = f'ถึงตำแหน่ง {pose_name} แล้ว'
            self.publish_status(success_msg)
        else:
            error_msg = f'การนำทางไปยัง {pose_name} ล้มเหลว'
            self.publish_status(error_msg)

def main():
    rclpy.init()
    name = sys.argv[1]
    node = AMRNav(name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_status("ระบบนำทางถูกปิด")
    finally:
        node.navigator.cancelTask()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
import tf2_ros
import numpy as np
import struct
import math

class RvizTestPublisher(Node):
    def __init__(self):
        super().__init__('rviz_test_pub')

        self.pc_pub = self.create_publisher(PointCloud2, 'points', 10)
        self.marker_pub = self.create_publisher(Marker, 'marker', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.img_pub = self.create_publisher(Image, 'image_raw', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        t = self.count * 0.1
        self.publish_pc(t)
        self.publish_markers(t)
        self.publish_scan(t)
        self.publish_odom(t)
        self.publish_map(t)
        self.publish_image(t)
        self.publish_tf(t)
        self.count += 1

    def publish_pc(self, t):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        points = []
        for i in range(100):
            r = i * 0.05
            angle = i * 0.2 + t
            points.append([r * math.cos(angle), r * math.sin(angle), math.sin(t + i * 0.1)])
        msg.height, msg.width = 1, len(points)
        msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 12, True
        msg.row_step = 12 * msg.width
        msg.data = b''.join([struct.pack('fff', *p) for p in points])
        self.pc_pub.publish(msg)

    def publish_markers(self, t):
        m = Marker()
        m.header.frame_id, m.header.stamp = "map", self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "test", 0, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 2.0 * math.cos(t), 2.0 * math.sin(t), 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color.r, m.color.a = 1.0, 1.0
        self.marker_pub.publish(m)

    def publish_scan(self, t):
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = self.get_clock().now().to_msg(), "base_link"
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, 0.1
        scan.range_min, scan.range_max = 0.0, 10.0
        scan.ranges = [3.0 + 0.5 * math.sin(t + i * 0.1) for i in range(int(2*math.pi/0.1))]
        self.scan_pub.publish(scan)

    def publish_odom(self, t):
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = self.get_clock().now().to_msg(), "map"
        odom.pose.pose.position.x, odom.pose.pose.position.y = 3.0 * math.cos(t*0.5), 3.0 * math.sin(t*0.5)
        self.odom_pub.publish(odom)

    def publish_map(self, t):
        m = OccupancyGrid()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), "map"
        m.info.resolution, m.info.width, m.info.height = 0.1, 50, 50
        m.info.origin.position.x, m.info.origin.position.y = -2.5, -2.5
        data = np.zeros((50, 50), dtype=np.int8)
        # Draw a moving square
        cx, cy = int(25 + 10 * math.cos(t)), int(25 + 10 * math.sin(t))
        data[max(0,cy-2):min(50,cy+2), max(0,cx-2):min(50,cx+2)] = 100
        m.data = data.flatten().tolist()
        self.map_pub.publish(m)

    def publish_image(self, t):
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.width, img.height = 40, 30
        img.encoding, img.step = "rgb8", 40 * 3
        data = np.zeros((30, 40, 3), dtype=np.uint8)
        cx, cy = int(20 + 10 * math.cos(t)), int(15 + 5 * math.sin(t))
        data[max(0,cy-3):min(30,cy+3), max(0,cx-3):min(40,cx+3)] = [0, 255, 0]
        img.data = data.flatten().tobytes()
        self.img_pub.publish(img)

    def publish_tf(self, t):
        ts = TransformStamped()
        ts.header.stamp, ts.header.frame_id, ts.child_frame_id = self.get_clock().now().to_msg(), "map", "base_link"
        ts.transform.translation.x, ts.transform.translation.y = 3.0 * math.cos(t*0.5), 3.0 * math.sin(t*0.5)
        ts.transform.rotation.z, ts.transform.rotation.w = math.sin(t*0.25), math.cos(t*0.25)
        self.tf_broadcaster.sendTransform(ts)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RvizTestPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

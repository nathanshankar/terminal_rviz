#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, LaserScan, Image, Range, Temperature, FluidPressure, Illuminance, RelativeHumidity, JointState
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerUpdate, InteractiveMarker
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, TransformStamped, WrenchStamped, PolygonStamped, PointStamped, PoseStamped, AccelStamped, TwistStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid, Path, GridCells
import tf2_ros
import numpy as np
import struct
import math

class AllPluginsTestPublisher(Node):
    def __init__(self):
        super().__init__('all_plugins_test_pub')

        self.pc_pub = self.create_publisher(PointCloud2, 'points', 10)
        self.marker_pub = self.create_publisher(Marker, 'marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'marker_array', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.img_pub = self.create_publisher(Image, 'image_raw', 10)
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.pose_array_pub = self.create_publisher(PoseArray, 'pose_array', 10)
        self.range_pub = self.create_publisher(Range, 'range', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, 'wrench', 10)
        self.polygon_pub = self.create_publisher(PolygonStamped, 'polygon', 10)
        self.grid_cells_pub = self.create_publisher(GridCells, 'grid_cells', 10)
        self.point_stamped_pub = self.create_publisher(PointStamped, 'point_stamped', 10)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, 'pose_stamped', 10)
        self.accel_pub = self.create_publisher(AccelStamped, 'accel', 10)
        self.twist_pub = self.create_publisher(TwistStamped, 'twist', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, 'pressure', 10)
        self.illuminance_pub = self.create_publisher(Illuminance, 'illuminance', 10)
        self.humidity_pub = self.create_publisher(RelativeHumidity, 'humidity', 10)
        self.effort_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.im_pub = self.create_publisher(InteractiveMarkerUpdate, 'interactive_markers_update', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def timer_callback(self):
        t = self.count * 0.1
        now = self.get_clock().now().to_msg()
        self.publish_pc(t, now)
        self.publish_markers(t, now)
        self.publish_scan(t, now)
        self.publish_odom(t, now)
        self.publish_map(t, now)
        self.publish_image(t, now)
        self.publish_tf(t, now)
        self.publish_path(t, now)
        self.publish_pose_array(t, now)
        self.publish_range(t, now)
        self.publish_scalars(t, now)
        self.publish_geometry(t, now)
        self.publish_extra(t, now)
        self.count += 1

    def publish_extra(self, t, now):
        js = JointState()
        js.header.stamp = now
        js.name = ['joint1', 'joint2']
        js.position = [math.sin(t), math.cos(t)]
        js.effort = [10.0 * math.sin(t), 5.0 * math.cos(t)]
        self.effort_pub.publish(js)

        imu = InteractiveMarkerUpdate()
        imu.server_id = "test_server"
        im = InteractiveMarker()
        im.header.frame_id, im.header.stamp = "map", now
        im.name, im.description = "test_im", "Interactive Marker"
        im.pose.position.z = 2.0
        im.pose.orientation.w = 1.0
        imu.markers.append(im)
        self.im_pub.publish(imu)

    def publish_pc(self, t, now):
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = now, 'map'
        pts = [[(i*0.05)*math.cos(i*0.2+t), (i*0.05)*math.sin(i*0.2+t), math.sin(t+i*0.1)] for i in range(100)]
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 12, True
        msg.row_step = 12 * msg.width
        msg.data = b''.join([struct.pack('fff', *p) for p in pts])
        self.pc_pub.publish(msg)

    def publish_markers(self, t, now):
        m = Marker()
        m.header.frame_id, m.header.stamp = "map", now
        m.ns, m.id, m.type, m.action = "test", 0, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 2.0*math.cos(t), 2.0*math.sin(t), 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color.r, m.color.g, m.color.a = 1.0, 0.5, 1.0
        self.marker_pub.publish(m)
        ma = MarkerArray()
        for i in range(3):
            m2 = Marker()
            m2.header.frame_id, m2.header.stamp = "map", now
            m2.ns, m2.id, m2.type, m2.action = "test_arr", i, Marker.SPHERE, Marker.ADD
            m2.pose.position.x, m2.pose.position.y, m2.pose.position.z = 4.0+math.cos(t+i), math.sin(t+i), 0.5
            m2.scale.x = m2.scale.y = m2.scale.z = 0.3
            m2.color.b, m2.color.a = 1.0, 1.0
            ma.markers.append(m2)
        self.marker_array_pub.publish(ma)

    def publish_scan(self, t, now):
        s = LaserScan()
        s.header.stamp, s.header.frame_id = now, "base_link"
        s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, 0.1
        s.range_min, s.range_max = 0.0, 10.0
        s.ranges = [3.0+0.5*math.sin(t+i*0.1) for i in range(int(2*math.pi/0.1))]
        self.scan_pub.publish(s)

    def publish_odom(self, t, now):
        o = Odometry()
        o.header.stamp, o.header.frame_id = now, "map"
        o.pose.pose.position.x, o.pose.pose.position.y = 3.0*math.cos(t*0.5), 3.0*math.sin(t*0.5)
        self.odom_pub.publish(o)

    def publish_map(self, t, now):
        m = OccupancyGrid()
        m.header.stamp, m.header.frame_id = now, "map"
        m.info.resolution, m.info.width, m.info.height = 0.1, 50, 50
        m.info.origin.position.x, m.info.origin.position.y = -2.5, -2.5
        d = np.zeros((50,50), dtype=np.int8)
        cx, cy = int(25+10*math.cos(t)), int(25+10*math.sin(t))
        d[max(0,cy-2):min(50,cy+2), max(0,cx-2):min(50,cx+2)] = 100
        m.data = d.flatten().tolist()
        self.map_pub.publish(m)

    def publish_image(self, t, now):
        img = Image()
        img.header.stamp, img.width, img.height = now, 40, 30
        img.encoding, img.step = "rgb8", 40*3
        d = np.zeros((30,40,3), dtype=np.uint8)
        cx, cy = int(20+10*math.cos(t)), int(15+5*math.sin(t))
        d[max(0,cy-3):min(30,cy+3), max(0,cx-3):min(40,cx+3)] = [0,255,0]
        img.data = d.flatten().tobytes()
        self.img_pub.publish(img)

    def publish_tf(self, t, now):
        ts = TransformStamped()
        ts.header.stamp, ts.header.frame_id, ts.child_frame_id = now, "map", "base_link"
        ts.transform.translation.x, ts.transform.translation.y = 3.0*math.cos(t*0.5), 3.0*math.sin(t*0.5)
        ts.transform.rotation.z, ts.transform.rotation.w = math.sin(t*0.25), math.cos(t*0.25)
        self.tf_broadcaster.sendTransform(ts)
        ts2 = TransformStamped()
        ts2.header.stamp, ts2.header.frame_id, ts2.child_frame_id = now, "base_link", "sensor"
        ts2.transform.translation.z, ts2.transform.rotation.w = 0.5, 1.0
        self.tf_broadcaster.sendTransform(ts2)

    def publish_path(self, t, now):
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = now, "map"
        p.pose.position.x, p.pose.position.y = 5.0*math.cos(t*0.2), 5.0*math.sin(t*0.2)
        self.path_msg.poses.append(p)
        if len(self.path_msg.poses) > 50: self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

    def publish_pose_array(self, t, now):
        pa = PoseArray()
        pa.header.stamp, pa.header.frame_id = now, "map"
        for i in range(5):
            p = Pose()
            p.position.x, p.position.y, p.orientation.w = -4.0+i*0.5, 2.0+math.sin(t+i), 1.0
            pa.poses.append(p)
        self.pose_array_pub.publish(pa)

    def publish_range(self, t, now):
        r = Range()
        r.header.stamp, r.header.frame_id = now, "sensor"
        r.radiation_type, r.field_of_view = Range.ULTRASOUND, 0.5
        r.min_range, r.max_range, r.range = 0.1, 5.0, 2.0+math.sin(t)
        self.range_pub.publish(r)

    def publish_scalars(self, t, now):
        tmp = Temperature()
        tmp.header.stamp, tmp.header.frame_id, tmp.temperature = now, "map", 25.0+5.0*math.sin(t)
        self.temp_pub.publish(tmp)
        prs = FluidPressure()
        prs.header.stamp, prs.header.frame_id, prs.fluid_pressure = now, "map", 1013.0+10.0*math.cos(t)
        self.pressure_pub.publish(prs)
        ill = Illuminance()
        ill.header.stamp, ill.header.frame_id, ill.illuminance = now, "map", 500.0+100.0*math.sin(t*0.5)
        self.illuminance_pub.publish(ill)
        hum = RelativeHumidity()
        hum.header.stamp, hum.header.frame_id, hum.relative_humidity = now, "map", 0.5+0.1*math.cos(t*0.3)
        self.humidity_pub.publish(hum)

    def publish_geometry(self, t, now):
        wr = WrenchStamped()
        wr.header.stamp, wr.header.frame_id = now, "base_link"
        wr.wrench.force.x, wr.wrench.torque.z = math.sin(t), math.cos(t)
        self.wrench_pub.publish(wr)
        poly = PolygonStamped()
        poly.header.stamp, poly.header.frame_id = now, "map"
        for x, y in [(-2.0,-2.0), (-1.0,-2.0), (-1.0,-1.0), (-2.0,-1.0)]:
            pt = Point32()
            pt.x, pt.y = float(x), float(y)
            poly.polygon.points.append(pt)
        self.polygon_pub.publish(poly)
        gc = GridCells()
        gc.header.stamp, gc.header.frame_id = now, "map"
        gc.cell_width = gc.cell_height = 0.2
        for i in range(3):
            pt = Point()
            pt.x, pt.y = -5.0+i*0.5, -5.0+math.sin(t+i)
            gc.cells.append(pt)
        self.grid_cells_pub.publish(gc)
        pts = PointStamped()
        pts.header.stamp, pts.header.frame_id = now, "map"
        pts.point.x, pts.point.y, pts.point.z = -1.0, -1.0, 2.0+math.sin(t)
        self.point_stamped_pub.publish(pts)
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = now, "map"
        ps.pose.position.x, ps.pose.position.y = 1.0, -1.0
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(t*0.5), math.cos(t*0.5)
        self.pose_stamped_pub.publish(ps)
        acc = AccelStamped()
        acc.header.stamp, acc.header.frame_id = now, "base_link"
        acc.accel.linear.x = math.sin(t)
        self.accel_pub.publish(acc)
        tw = TwistStamped()
        tw.header.stamp, tw.header.frame_id, tw.twist.angular.z = now, "base_link", 1.0
        self.twist_pub.publish(tw)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AllPluginsTestPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

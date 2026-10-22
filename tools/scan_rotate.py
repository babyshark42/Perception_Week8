#!/usr/bin/env python3
"""ชดเชยมุมเอียงของ LiDAR บน PC (แทนการแก้ URDF บนหุ่น): /scan -> /scan_fixed

LiDAR ติดเอียงไปทางซ้าย φ องศา (calibrate_odom.py: +1.8°) -> จุดที่มุม a ใน scan จริง ๆ อยู่ที่ a + φ ในกรอบหุ่น
จึงเลื่อน angle_min/angle_max ไป +φ แล้วใช้ frame base_scan เดิม (ซึ่ง TF ถือว่าตรงกับหุ่น)
ใช้กับ params/scanfix.yaml (AMCL + costmap อ่าน /scan_fixed) — start_nav.sh scanfix เปิดให้เอง
ใช้:  python3 tools/scan_rotate.py --deg 1.8
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deg', type=float, default=1.8)
    args = ap.parse_args()
    rclpy.init()
    node = Node('scan_rotate')
    phi = math.radians(args.deg)
    pub = node.create_publisher(LaserScan, '/scan_fixed', 10)

    def cb(m):
        m.angle_min += phi
        m.angle_max += phi
        pub.publish(m)

    node.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)
    node.get_logger().info(f'/scan -> /scan_fixed  rotate {args.deg:+.2f} deg')
    rclpy.spin(node)


if __name__ == '__main__':
    main()

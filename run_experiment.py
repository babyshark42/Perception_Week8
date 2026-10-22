#!/usr/bin/python3 -s
"""FRA503 Assignment 8: ผลของระยะทางต่อความแม่นของ Nav2 (TurtleBot3 Burger หุ่นจริง) — รันอัตโนมัติ

ตัวแปรต้น: ระยะทาง 4 ระยะ (เส้นตรงตามแนวเทป) — ตัวแปรอื่นตรึงไว้ทั้งหมด (ดู README.md)
ลำดับรอบสุ่มครั้งเดียวแล้วเก็บไว้ใน results/<condition>/schedule.csv (รันต่อได้ถ้าหยุดกลางคัน)

ตำแหน่งจริง (ground truth) = LiDAR ICP เทียบ scan อ้างอิงตอนหุ่นอยู่บนเทปจุดเริ่ม (กรอบเทป: x ตามเทป, y ซ้าย)
  scan อ้างอิงดึงจาก bag ปรับเทียบ (ช่วงนิ่งแรก = ตอนคนวางหุ่นบนเทป) เก็บไว้ที่ results/ref_scan.npz
  ใช้ /scan ดิบ + มุม LiDAR จริง (SCAN_YAW_DEG) เสมอ ไม่ขึ้นกับ TF/relay ที่ Nav2 ใช้ -> เทียบทุก condition ได้

แต่ละรอบ: หุ่นกลับจุดเริ่มเอง (ICP นำทาง) -> วัดจุดเริ่มจริง -> ตั้ง initial pose (มี covariance) + AMCL อัปเดต
  -> เป้า = จุดเริ่มที่ AMCL เห็น + d ตามแนวเทป -> Nav2 วิ่ง -> บันทึก TF / AMCL / odom -> วัดจุดจอดจริงด้วย ICP

ผลลัพธ์:
  results/<condition>/trials.csv        1 แถวต่อ 1 รอบ (error ทุกตัวอยู่ในกรอบเทป)
  results/<condition>/traj/<trial>.csv  trajectory (amcl + odom)

ต้องเปิด bringup บนหุ่น และ ./start_nav.sh <params> ไว้ก่อน
ใช้:  ./run_experiment.py --condition baseline                 (ข้อ 1)
      ./run_experiment.py --condition scanfix --scan-fixed    (ข้อ 2-C: ./start_nav.sh scanfix หรือแก้ URDF)
      ./run_experiment.py --condition amclfix                 (ข้อ 2-A: ./start_nav.sh amclfix)
      ./run_experiment.py --condition tuned --scan-fixed --start 0 0 --init-yaw 0   (./start_nav.sh tuned)
      --manual  ให้คนวางหุ่นเองแทนการกลับจุดเริ่มอัตโนมัติ
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from bag_calibrate import Reference, icp, scan_xy, median_scan, compose, inverse, wrap, read_bag, segments  # noqa: E402

# ---------------- ตั้งค่าการทดลอง (ตัวแปรควบคุม — ห้ามเปลี่ยนระหว่างข้อ 1 กับข้อ 2) ----------------
DISTANCES = [0.25, 0.50, 0.75, 1.00]  # m  ตัวแปรต้น (เป้าไกลสุดห่างผิวกำแพง ~31 cm)
REPEATS = 5
SEED = 8                              # seed เดียวกันทุก condition -> ลำดับรอบเหมือนกัน
SPEED = 0.10                          # m/s
MAX_W = 1.0                           # rad/s
START = (0.10, -0.40)                 # พิกัด map ของจุดเริ่มบนเทป (ใช้เป็นค่าเดาแรกของ initial pose)
# LiDAR ติดเอียงจากแกนหุ่น (calibrate_odom.py บนหุ่นจริง 14 ต.ค. 2569: +1.83° ± 0.48, n = 14)
# ถ้า TF/scan ยังไม่ชดเชย ทิศที่ AMCL บอก = ทิศ LiDAR ไม่ใช่ทิศหุ่น
SCAN_YAW_DEG = 1.8
SCAN_X = -0.032                       # base_footprint -> base_scan
INIT_COV_XY = 0.03 ** 2               # initial pose: σ 3 cm, 3° ให้ AMCL ปรับเข้าหากำแพงเองได้
INIT_COV_YAW = math.radians(3.0) ** 2
TIMEOUT_S = 60.0
MIN_BATT_V = 11.5                     # ต่ำกว่านี้หยุด (รันต่อได้หลังชาร์จ)
REST_EVERY = 10                       # พัก LiDAR ทุก ๆ กี่รอบ
REST_MIN = 5
HOME_TOL = 0.012                      # m   กลับจุดเริ่มต้องคลาดไม่เกินนี้
HOME_TOL_YAW = math.radians(0.6)
STOP_FRONT, STOP_BACK = 0.18, 0.08    # m จาก LiDAR ในช่องทางวิ่ง -> หยุดฉุกเฉิน
REF_BAG = os.path.join(ROOT, 'results', 'calib_bags', 'calib_20261014_171621')
REF_PATH = os.path.join(ROOT, 'results', 'ref_scan.npz')
# ---------------------------------------------------------------------------------------------


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z, p.pose.orientation.w = yaw_to_quat(yaw)
    return p


def to_tape(p_map, origin, th):
    """จุดใน map -> กรอบเทป"""
    dx, dy = p_map[0] - origin[0], p_map[1] - origin[1]
    c, s = math.cos(th), math.sin(th)
    return c * dx + s * dy, -s * dx + c * dy


class SafetyStop(Exception):
    pass


class Logger(Node):
    """เก็บ /amcl_pose, /odom, /scan, /battery_state, อ่าน TF และส่ง /cmd_vel ตอนพาหุ่นกลับจุดเริ่ม"""

    def __init__(self):
        super().__init__('fra503_w8_logger')
        self.lock = threading.Lock()
        self.amcl = None
        self.amcl_t = 0.0
        self.amcl_count = 0
        self.odom = None
        self.scan = None
        self.scan_seq = 0
        self.batt_v = float('nan')
        self.recording = False
        self.traj = []
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.on_batt, qos_profile_sensor_data)
        self.param_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_pub = None
        self.stamped = False
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.markers = MarkerArray()
        self.create_timer(0.5, lambda: self.marker_pub.publish(self.markers))
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

    # ---------- callbacks ----------
    def on_amcl(self, m):
        p = m.pose.pose
        with self.lock:
            self.amcl = (p.position.x, p.position.y, quat_to_yaw(p.orientation))
            self.amcl_t = time.time()
            if self.recording:
                self.amcl_count += 1
                self.traj.append((time.time(), 'amcl', *self.amcl, 0.0, 0.0))

    def on_odom(self, m):
        p = m.pose.pose
        t = m.twist.twist
        with self.lock:
            self.odom = (p.position.x, p.position.y, quat_to_yaw(p.orientation))
            self.odom_v = (t.linear.x, t.angular.z)
            if self.recording:
                self.traj.append((time.time(), 'odom', *self.odom, t.linear.x, t.angular.z))

    def on_scan(self, m):
        with self.lock:
            self.scan = m
            self.scan_seq += 1

    def on_batt(self, m):
        self.batt_v = m.voltage

    def get(self):
        with self.lock:
            return self.odom, self.scan, self.scan_seq

    # ---------- cmd_vel ----------
    def setup_cmd(self):
        types = {i.topic_type for i in self.get_subscriptions_info_by_topic('/cmd_vel')}
        if not types:
            return False
        self.stamped = 'geometry_msgs/msg/TwistStamped' in types
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist, '/cmd_vel', 10)
        return True

    def send(self, v, w=0.0):
        if self.stamped:
            t = TwistStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            tw = t.twist
        else:
            t = tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def corridor(self, forward=True, half_width=0.12):
        m = self.scan
        if m is None:
            return float('inf')
        a = m.angle_min + m.angle_increment * np.arange(len(m.ranges))
        r = np.asarray(m.ranges, float)
        ok = np.isfinite(r) & (r > 0.05)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        sel = (np.abs(y) < half_width) & ((x > 0) if forward else (x < 0))
        return float(np.abs(x[sel]).min()) if sel.any() else float('inf')

    # ---------- Nav2 helpers ----------
    def start_recording(self):
        with self.lock:
            self.traj = []
            self.amcl_count = 0
            self.recording = True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            return list(self.traj), self.amcl_count

    def set_initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_to_quat(yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = INIT_COV_XY
        cov[35] = INIT_COV_YAW
        m.pose.covariance = cov
        self.init_pub.publish(m)

    def map_pose(self):
        """ตำแหน่งที่ Nav2 ใช้จริง = TF map->base_footprint"""
        try:
            tr = self.tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform
        return t.translation.x, t.translation.y, quat_to_yaw(t.rotation)

    def wait_map_tf(self, x, y, timeout=15.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.map_pose()
            if p and math.dist(p[:2], (x, y)) < 0.15:
                return True
            time.sleep(0.1)
        return False

    def _call(self, cli, req, what):
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{what}: service not available')
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not fut.done():
            raise RuntimeError(f'{what}: timeout')
        return fut.result()

    def fresh_amcl(self, timeout=5.0):
        cli = self.create_client(Empty, '/request_nomotion_update')
        t_req = time.time()
        for _ in range(3):
            self._call(cli, Empty.Request(), 'request_nomotion_update')
            time.sleep(0.3)
        self.destroy_client(cli)
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if self.amcl_t > t_req:
                    return self.amcl
            time.sleep(0.05)
        print('  WARN: amcl_pose ไม่อัปเดต ใช้ค่าล่าสุด')
        return self.amcl

    def set_limits(self):
        def dparam(name, val):
            return Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(val)))
        req = SetParameters.Request()
        req.parameters = [dparam('FollowPath.max_vel_x', SPEED), dparam('FollowPath.max_speed_xy', SPEED),
                          dparam('FollowPath.max_vel_theta', MAX_W)]
        res = self._call(self.param_cli, req, 'set limits')
        if not all(r.successful for r in res.results):
            raise RuntimeError('set limits failed')

    def show_goal(self, start, goal, label):
        ms = [Marker(action=Marker.DELETEALL)]
        pts = [(*start, (0.1, 0.9, 0.1), 'start'), (*goal, (0.9, 0.1, 0.1), 'goal')]
        for i, (x, y, yaw, rgb, name) in enumerate(pts):
            for kind in (Marker.ARROW, Marker.TEXT_VIEW_FACING):
                m = Marker()
                m.header.frame_id = 'map'
                m.ns = 'fra503_w8'
                m.id = 2 * i + (kind == Marker.TEXT_VIEW_FACING)
                m.type = kind
                m.pose.position.x, m.pose.position.y = x, y
                m.pose.orientation.z, m.pose.orientation.w = yaw_to_quat(yaw)
                m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
                if kind == Marker.ARROW:
                    m.scale.x, m.scale.y, m.scale.z = 0.15, 0.03, 0.03
                else:
                    m.pose.position.z = 0.12
                    m.scale.z = 0.06
                    m.text = f'{name} ({x:.2f},{y:.2f})' + (f'  {label}' if name == 'goal' else '')
                ms.append(m)
        self.markers = MarkerArray(markers=ms)


class Truth:
    """ตำแหน่งจริงของหุ่นในกรอบเทป จาก ICP ของ /scan ดิบเทียบ scan อ้างอิง (+ odom ระหว่าง scan)"""

    def __init__(self, log, ref):
        self.log = log
        self.ref = Reference(scan_xy(*ref))
        self.X = (SCAN_X, 0.0, math.radians(SCAN_YAW_DEG))   # มุม LiDAR จริง
        self.pose = None
        self.pose_odom = None
        self.seq = -1

    def _icp(self, a, r, rmin, rmax, guess, iters=40):
        X = self.X
        s, n = icp(self.ref, scan_xy(a, r, rmin, rmax), compose(compose(inverse(X), guess), X), iters=iters)
        if s is None:
            return None
        return compose(compose(X, s), inverse(X))

    def predict(self):
        odom, _, _ = self.log.get()
        if self.pose is None:
            return (0.0, 0.0, 0.0)
        if self.pose_odom is None or odom is None:
            return self.pose
        return compose(self.pose, compose(inverse(self.pose_odom), odom))

    def track(self):
        """เรียกถี่ ๆ ระหว่างขับ: ถ้ามี scan ใหม่ทำ ICP แล้วคืน pose ปัจจุบัน"""
        odom, m, seq = self.log.get()
        if m is not None and seq != self.seq:
            self.seq = seq
            a = m.angle_min + m.angle_increment * np.arange(len(m.ranges))
            p = self._icp(a, np.asarray(m.ranges, float), m.range_min, m.range_max, self.predict(), iters=25)
            if p is not None:
                self.pose, self.pose_odom = p, odom
        return self.predict()

    def still_pose(self, n=5, settle=0.8):
        """หุ่นนิ่ง: median ของ n scan ใหม่ -> ICP ละเอียด"""
        for _ in range(int(settle / 0.05)):
            self.log.send(0.0)
            time.sleep(0.05)
        group, last = [], self.log.get()[2]
        t0 = time.time()
        while len(group) < n and time.time() - t0 < 5.0:
            _, m, seq = self.log.get()
            if seq != last:
                last = seq
                a = m.angle_min + m.angle_increment * np.arange(len(m.ranges))
                group.append((0.0, a, np.asarray(m.ranges, float), m.range_min, m.range_max))
            time.sleep(0.02)
        scan = median_scan(group)
        if self.pose is None:
            # ยังไม่รู้ตำแหน่ง (เพิ่งเปิดสคริปต์ หุ่นอาจค้างอยู่กลางทาง): ลองเดาหลายจุดตามแนวเทป เลือกที่จับคู่กำแพงได้มากสุด
            guesses = [(x, y, math.radians(a)) for x in (0.0, 0.25, 0.5, 0.75, 1.0, 1.2)
                       for y in (0.0, -0.15, -0.3) for a in (0, -20, 20, -40, 40)]
            # ห้องสี่เหลี่ยมสมมาตรเมื่อหมุน 180° -> ไม่เดา 180° และรับเฉพาะคำตอบที่หันไปตามเทป (|yaw| < 90°)
        else:
            guesses = [self.predict()]
        best = None
        for g in guesses:
            X = self.X
            r, n = icp(self.ref, scan_xy(*scan), compose(compose(inverse(X), g), X), iters=60)
            if r is None:
                continue
            body = compose(compose(X, r), inverse(X))
            if self.pose is None and abs(body[2]) > math.pi / 2:
                continue
            if best is None or n > best[1]:
                best = (body, n)
        p = best[0] if best else None
        if p is None:
            raise RuntimeError('ICP ไม่ลู่เข้า (มีคน/ของในห้อง?)')
        self.pose, self.pose_odom = p, self.log.get()[0]
        return p


def stop_robot(log, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            log.send(0.0)
        except Exception:
            return
        time.sleep(0.05)


def rotate_to(log, truth, th_des, tol=HOME_TOL_YAW, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = truth.track()
        e = wrap(th_des - p[2])
        if abs(e) < tol:
            break
        log.send(0.0, math.copysign(min(0.4, max(0.12, 1.2 * abs(e))), e))
        time.sleep(0.04)
    log.send(0.0)


def drive_to(log, truth, target, reverse, timeout=60.0):
    """ขับตรงเข้าหาจุด target (กรอบเทป) แก้ทิศไปด้วย หยุดเมื่อถึง/ผ่านจุดหมาย"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        x, y, th = truth.track()
        dx, dy = target[0] - x, target[1] - y
        along = math.cos(th) * dx + math.sin(th) * dy      # ระยะที่เหลือตามแกนหุ่น (+ = อยู่ข้างหน้า)
        if reverse:
            along = -along
        if along < 0.003:
            break
        if log.corridor(forward=not reverse) < (STOP_BACK if reverse else STOP_FRONT):
            stop_robot(log, 0.5)
            raise SafetyStop('มีสิ่งกีดขวางในทางกลับจุดเริ่ม')
        bearing = math.atan2(dy, dx) + (math.pi if reverse else 0.0)
        e = wrap(bearing - th) if math.hypot(dx, dy) > 0.05 else 0.0
        v = min(0.06, max(0.015, 0.8 * along))
        log.send(-v if reverse else v, max(-0.3, min(0.3, 1.5 * e)))
        time.sleep(0.04)
    log.send(0.0)


def fine_rotate(log, truth, th_des=0.0, tries=5, w=0.15):
    """หมุนแบบพัลส์: สั่ง w คงที่นาน |err|/w แล้วหยุดวัดใหม่ — ไม่ไวต่อความหน่วงของคำสั่ง (WiFi ~0.3-0.9 s)"""
    p = truth.still_pose()
    for _ in range(tries):
        e = wrap(th_des - p[2])
        if abs(e) < HOME_TOL_YAW:
            break
        t_end = time.time() + abs(e) / w
        while time.time() < t_end:
            log.send(0.0, math.copysign(w, e))
            time.sleep(0.04)
        p = truth.still_pose(settle=1.0)
    return p


HOME_WP = (0.30, 0.0)   # จุดพักหน้าจุดเริ่ม: ตั้งหัวตรงที่นี่ (ห่างกำแพงพอหมุน) แล้วถอยตรงเข้าจุดเริ่ม


def go_home(log, truth):
    """พาหุ่นกลับจุดเริ่ม (0,0,0) ของกรอบเทปด้วย ICP แบบถอยจอด คืน pose จริงตอนนิ่ง"""
    p = truth.still_pose()
    for _ in range(3):
        if math.hypot(p[0], p[1]) < HOME_TOL and abs(p[2]) < HOME_TOL_YAW:
            return p
        dw = math.hypot(HOME_WP[0] - p[0], HOME_WP[1] - p[1])
        if dw > 0.02:
            bearing = math.atan2(HOME_WP[1] - p[1], HOME_WP[0] - p[0])
            reverse = math.cos(wrap(bearing - p[2])) < 0
            if dw > 0.04:
                rotate_to(log, truth, wrap(bearing + (math.pi if reverse else 0.0)), tol=math.radians(2.0))
            drive_to(log, truth, HOME_WP, reverse)
        rotate_to(log, truth, 0.0, tol=math.radians(2.0))
        drive_to(log, truth, (0.0, 0.0), reverse=True)
        p = fine_rotate(log, truth, 0.0)
    return p


WHEEL_HALF = 0.08   # m  ครึ่งระยะห่างล้อ Burger (0.160 / 2)


def wait_file(path, msg):
    print(msg, flush=True)
    while not os.path.exists(path):
        time.sleep(0.5)
    with open(path) as f:
        txt = f.read()
    os.remove(path)
    return txt


def rel(mark, p):
    return compose(inverse(mark), p)


def tape_check(out_dir, row, mark, end, st, odom_p, tf_p, amcl_p):
    """รอค่าตลับเมตร (เทียบเครื่องหมายตอนเริ่ม) แล้วเทียบกับ ICP / odom / AMCL / Nav2 (ทุกค่าเทียบตำแหน่งที่ทำเครื่องหมาย)"""
    while True:
        txt = wait_file(os.path.join(out_dir, 'tape.txt'),
                        f'WAIT_TAPE {row["trial"]}: วัดล้อซ้าย/ขวาเทียบเครื่องหมาย "dxL dyL dxR dyR" หรือกึ่งกลาง "dx dy" (cm)')
        try:
            v = [float(t) for t in txt.replace(',', ' ').split()]
        except ValueError:
            v = []
        if len(v) in (2, 4):
            break
        print('  ต้องมี 2 หรือ 4 ตัวเลข', flush=True)
    nan = float('nan')
    if len(v) == 4:
        lx, ly = v[0] / 100, WHEEL_HALF + v[1] / 100
        rx_, ry_ = v[2] / 100, -WHEEL_HALF + v[3] / 100
        tape = ((lx + rx_) / 2, (ly + ry_) / 2, math.atan2(ly - ry_, lx - rx_) - math.pi / 2)
    else:
        tape = (v[0] / 100, v[1] / 100, nan)
    srcs = {'icp': rel(mark, end), 'odom': rel(mark, odom_p), 'nav2': rel(mark, tf_p), 'amcl': rel(mark, amcl_p)}
    out = {'condition': row['condition'], 'trial': row['trial'], 'dist': row['dist'], 'result': row['result'],
           'tape_dx_cm': round(tape[0] * 100, 2), 'tape_dy_cm': round(tape[1] * 100, 2),
           'tape_yaw_deg': round(math.degrees(tape[2]), 2) if not math.isnan(tape[2]) else nan,
           'start_off_cm': round(math.hypot(*rel(mark, st)[:2]) * 100, 2)}
    for k, p in srcs.items():
        out[f'{k}_dx_cm'] = round(p[0] * 100, 2)
        out[f'{k}_dy_cm'] = round(p[1] * 100, 2)
        out[f'{k}_yaw_deg'] = round(math.degrees(p[2]), 2)
        out[f'{k}_err_cm'] = round(math.hypot(p[0] - tape[0], p[1] - tape[1]) * 100, 2)
    path = os.path.join(out_dir, 'tape_check.csv')
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(out.keys()))
        if new:
            w.writeheader()
        w.writerow(out)
    print(f'  ตลับเมตร ({out["tape_dx_cm"]:.1f}, {out["tape_dy_cm"]:+.1f}) cm | คลาดจากตลับเมตร: ICP {out["icp_err_cm"]:.1f}'
          f'  odom {out["odom_err_cm"]:.1f}  AMCL {out["amcl_err_cm"]:.1f}  Nav2 เชื่อ {out["nav2_err_cm"]:.1f} cm', flush=True)


def load_reference():
    if os.path.exists(REF_PATH):
        z = np.load(REF_PATH)
        return z['a'], z['r'], float(z['rmin']), float(z['rmax'])
    print(f'สร้าง scan อ้างอิงจาก {REF_BAG} (ช่วงนิ่งแรก = ตอนคนวางหุ่นบนเทป)')
    odom, scans, _, _ = read_bag(REF_BAG)
    a0, a1, _, _ = segments(odom)[0]
    group = [s for s in scans if a0 <= s[0] <= a1]
    a, r, rmin, rmax = median_scan(group)
    np.savez(REF_PATH, a=a, r=r, rmin=rmin, rmax=rmax, n=len(group), bag=REF_BAG)
    print(f'  ใช้ {len(group)} scan -> {REF_PATH}')
    return a, r, rmin, rmax


def build_schedule(path, repeats, dists=None):
    if os.path.exists(path):
        with open(path) as f:
            return [(int(r['order']), float(r['dist']), int(r['rep'])) for r in csv.DictReader(f)]
    plan = [(d, r) for d in (dists or DISTANCES) for r in range(1, repeats + 1)]
    random.Random(SEED).shuffle(plan)
    sched = [(i + 1, d, r) for i, (d, r) in enumerate(plan)]
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['order', 'dist', 'rep'])
        w.writerows(sched)
    return sched


FIELDS = ['trial', 'condition', 'order', 'dist', 'rep', 'result', 'moved_ok', 'duration_s', 'recoveries',
          'amcl_updates', 'battery_v', 'scan_fixed',
          'start_x', 'start_y', 'start_yaw_deg',          # จุดเริ่มจริง (ICP, กรอบเทป) ควร ~0
          'tape_yaw_deg', 'origin_x', 'origin_y',         # แนวเทป/จุดเริ่ม ในพิกัด map (จาก AMCL ตอนเริ่ม)
          'goal_x', 'goal_y', 'init_err',
          'real_x', 'real_y', 'real_yaw_deg',             # จุดจอดจริง (ICP, กรอบเทป)
          'tf_x', 'tf_y', 'tf_yaw_deg',                   # Nav2 เชื่อ (TF) ตอนหยุด -> กรอบเทป
          'amcl_x', 'amcl_y', 'amcl_yaw_deg',             # AMCL หลังบังคับอัปเดต -> กรอบเทป
          'odom_x', 'odom_y', 'odom_yaw_deg',             # odom -> กรอบเทป (ยึดจุดเริ่มจริง)
          'e_stop', 'e_stop_long', 'e_stop_lat',          # จริง - เป้า (d, 0)
          'e_tf', 'e_amcl', 'e_odom',                     # |ที่เชื่อ - จริง|
          'e_odom_long', 'e_odom_lat', 'odom_scale',
          'e_yaw_real_deg', 'e_yaw_tf_deg', 'e_yaw_odom_deg',
          'home_err', 'odom_path_len', 'max_vx']


def r4(v):
    return round(v, 4) if v is not None and not math.isnan(v) else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--condition', required=True,
                    help='ชื่อชุดการทดลอง เช่น baseline / scanfix / amclfix (ใช้เป็นชื่อโฟลเดอร์ผล)')
    ap.add_argument('--repeats', type=int, default=REPEATS)
    ap.add_argument('--scan-fixed', action='store_true',
                    help='Nav2 ได้ scan/TF ที่ชดเชยมุม LiDAR แล้ว (start_nav.sh scanfix หรือแก้ URDF)')
    ap.add_argument('--manual', action='store_true', help='คนวางหุ่นที่จุดเริ่มเองทุกรอบ')
    ap.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'),
                    help='พิกัด map ของจุดเริ่ม (ค่าเดาแรก) ค่าเดิม 0.10 -0.40 ของ square_room; room_icp ใช้ 0 0')
    ap.add_argument('--init-yaw', type=float, metavar='DEG', help='yaw เริ่มที่ AMCL ควรเห็น (แทนค่าที่คำนวณเอง)')
    ap.add_argument('--max-trials', type=int, default=0, help='ทำแค่นี้รอบแล้วหยุด (0 = ทั้งหมด)')
    ap.add_argument('--tape-wait', action='store_true',
                    help='ตรวจด้วยตลับเมตร: เก็บผลใน results/tape_check/<condition>/ หยุดรอให้ทำเครื่องหมายตอนเริ่ม'
                         ' (ไฟล์ go.txt) และรอค่าตลับเมตรหลังจอดทุกรอบ (ไฟล์ tape.txt)')
    args = ap.parse_args()

    out_dir = os.path.join(ROOT, 'results', *(['tape_check'] if args.tape_wait else []), args.condition)
    os.makedirs(os.path.join(out_dir, 'traj'), exist_ok=True)
    csv_path = os.path.join(out_dir, 'trials.csv')
    sched = build_schedule(os.path.join(out_dir, 'schedule.csv'), args.repeats)  # --tape-wait: ระยะละ --repeats รอบ
    done = set()
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            done = {r['trial'] for r in csv.DictReader(f) if r['result'] != 'INVALID' and r.get('moved_ok') != '0'}

    ref = load_reference()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C -> KeyboardInterrupt, context ยังใช้ได้ -> สั่งหยุดหุ่นได้
    nav = BasicNavigator()
    log = Logger()
    ex = SingleThreadedExecutor()
    ex.add_node(log)
    threading.Thread(target=ex.spin, daemon=True).start()
    while log.get()[0] is None or log.get()[1] is None or not log.setup_cmd():
        print('รอ /odom /scan และ /cmd_vel ของหุ่น ...', flush=True)
        time.sleep(0.5)
    truth = Truth(log, ref)

    phi_u = 0.0 if args.scan_fixed else math.radians(SCAN_YAW_DEG)   # มุม LiDAR ที่ Nav2 ยังไม่รู้
    # map สร้างตอน TF ยังไม่ชดเชย: map x ขนาน LiDAR ตอนหุ่นอยู่บนเทป -> หุ่นจริงหัน -φ ใน map
    init_yaw = 0.0 if not args.scan_fixed else -math.radians(SCAN_YAW_DEG)
    if args.init_yaw is not None:      # เช่น room_icp: แผนที่สร้างในกรอบเทป -> หุ่นบนเทปหัน 0°
        init_yaw = math.radians(args.init_yaw)
    print(f'[{args.condition}] LiDAR {"ชดเชยแล้ว" if args.scan_fixed else f"ยังไม่ชดเชย ({SCAN_YAW_DEG}°)"}'
          f'  initial yaw {math.degrees(init_yaw):+.2f}°', flush=True)

    print('รอ Nav2 พร้อม...', flush=True)
    nav._waitForNodeToActivate('amcl')
    # bt_navigator จะ active หลัง costmap ได้ TF map->base ซึ่งต้องมี initial pose ก่อน
    _s0 = tuple(args.start) if args.start else START
    for _ in range(3):
        log.set_initial_pose(_s0[0], _s0[1], init_yaw)
        if log.wait_map_tf(_s0[0], _s0[1], timeout=5.0):
            break
    nav._waitForNodeToActivate('bt_navigator')

    todo = [s for s in sched if f'd{s[1]:.2f}_r{s[2]}' not in done]
    n_left = len(todo)
    if args.max_trials:
        todo = todo[:args.max_trials]
    print(f'ทั้งหมด {len(sched)} รอบ, ทำแล้ว {len(sched) - n_left}, รอบนี้ทำ {len(todo)}', flush=True)
    start0 = tuple(args.start) if args.start else START
    tape0 = init_yaw - phi_u                      # แนวเทปที่คาดไว้ (ใช้เดา initial pose ทุกรอบ ไม่ต่อจากรอบก่อน)
    since_rest = 0
    nan = float('nan')
    mark = None

    try:
        for order, d, rep in todo:
            name = f'd{d:.2f}_r{rep}'
            print(f'\n[{order}/{len(sched)}] {name}  ({time.strftime("%H:%M:%S")})', flush=True)
            if since_rest >= REST_EVERY:
                print(f'  พัก LiDAR {REST_MIN} นาที', flush=True)
                time.sleep(REST_MIN * 60)
                since_rest = 0
            if not math.isnan(log.batt_v) and log.batt_v < MIN_BATT_V:
                print(f'  แบต {log.batt_v:.2f} V < {MIN_BATT_V} V หยุด (ชาร์จแล้วรันคำสั่งเดิมต่อได้)')
                break

            # ---- กลับจุดเริ่ม ----
            if args.manual:
                if input('  วางหุ่นบนเทปจุดเริ่ม แล้วกด Enter (q=จบ): ').strip().lower() == 'q':
                    break
                st = truth.still_pose()
            else:
                st = go_home(log, truth)
            mark_file = os.path.join(ROOT, 'results', 'tape_check', 'mark.json')
            if args.tape_wait and mark is None and os.path.exists(mark_file):
                with open(mark_file) as f:
                    mj = json.load(f)
                mark = (mj['x'], mj['y'], math.radians(mj['yaw_deg']))   # ใช้เครื่องหมายเดิมที่พื้น
                print(f'  ใช้เครื่องหมายเดิม {mark_file}', flush=True)
            if args.tape_wait and mark is None:
                mark = st
                wait_file(os.path.join(out_dir, 'go.txt'),
                          f'WAIT_MARK หุ่นอยู่ที่จุดเริ่ม ({st[0] * 100:+.1f},{st[1] * 100:+.1f}) cm {math.degrees(st[2]):+.2f}°'
                          ' -> ทำเครื่องหมายที่พื้นข้างล้อซ้ายและขวา (แนวเพลาล้อ) แล้วสร้าง go.txt')
            home_err = math.hypot(st[0], st[1])
            if home_err > 0.02 or abs(st[2]) > math.radians(1.5):
                print('  WARN: กลับจุดเริ่มได้ไม่ถึงเกณฑ์ (บันทึกค่าจริงไว้ใน start_x/y/yaw)', flush=True)
            print(f'  จุดเริ่มจริง ({st[0] * 100:+.1f}, {st[1] * 100:+.1f}) cm  {math.degrees(st[2]):+.2f}°', flush=True)

            # ---- initial pose + AMCL ---- (เดาจากค่าคงที่ทุกรอบ: ถ้าต่อจาก AMCL รอบก่อน ค่าที่หลงจะสะสม)
            c, s = math.cos(tape0), math.sin(tape0)
            gx0, gy0 = start0[0] + c * st[0] - s * st[1], start0[1] + s * st[0] + c * st[1]
            yaw0 = tape0 + st[2] + phi_u
            amcl0 = None
            for attempt in range(3):
                log.set_initial_pose(gx0, gy0, yaw0)
                if not log.wait_map_tf(gx0, gy0, timeout=5.0):
                    continue
                log.fresh_amcl()
                a = log.fresh_amcl()
                if a and math.dist(a[:2], (gx0, gy0)) < 0.06 and abs(wrap(a[2] - yaw0)) < math.radians(4.0):
                    amcl0 = a
                    break
                if a:
                    print(f'  AMCL ลู่เข้าผิดที่ ({a[0]:.3f},{a[1]:.3f}) {math.degrees(a[2]):+.1f}° ตั้ง initial pose ใหม่', flush=True)
            if amcl0 is None:
                print('  ERROR: AMCL ไม่ลู่เข้าใกล้จุดเริ่ม ข้ามรอบนี้')
                continue
            init_err = math.dist(amcl0[:2], (gx0, gy0))
            # แนวเทปใน map ตามที่ AMCL เห็น: ทิศหุ่น = AMCL yaw - มุม LiDAR ที่ยังไม่ชดเชย, หักมุมเริ่มจริง
            tape = wrap(amcl0[2] - phi_u - st[2])
            c, s = math.cos(tape), math.sin(tape)
            origin = (amcl0[0] - (c * st[0] - s * st[1]), amcl0[1] - (s * st[0] + c * st[1]))
            gx, gy = origin[0] + c * d, origin[1] + s * d
            log.show_goal((*origin, tape), (gx, gy, tape), name)
            print(f'  AMCL เริ่ม ({amcl0[0]:.3f},{amcl0[1]:.3f}) yaw {math.degrees(amcl0[2]):+.2f}°'
                  f'  แนวเทป {math.degrees(tape):+.2f}°  เป้า ({gx:.3f},{gy:.3f})', flush=True)
            nav.clearAllCostmaps()
            log.set_limits()
            time.sleep(0.5)
            batt = log.batt_v

            # ---- วิ่ง ----
            odom0 = log.get()[0]
            log.start_recording()
            t0 = time.time()
            nav.goToPose(make_pose(nav, gx, gy, tape + phi_u))
            recoveries = 0
            canceled = False
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    recoveries = max(recoveries, getattr(fb, 'number_of_recoveries', 0))
                if time.time() - t0 > TIMEOUT_S and not canceled:
                    print('  timeout -> cancel', flush=True)
                    nav.cancelTask()
                    canceled = True
                time.sleep(0.05)
            duration = time.time() - t0
            tf_pose = log.map_pose()
            traj, amcl_updates = log.stop_recording()
            result = {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.CANCELED: 'CANCELED',
                      TaskResult.FAILED: 'FAILED'}.get(nav.getResult(), 'UNKNOWN')
            if result == 'SUCCEEDED' and duration < 0.5 * d / SPEED:
                result = 'INVALID'   # Nav2 humble: TF ไม่พร้อมแล้วตอบ "Reached the goal" ทั้งที่ไม่ได้วิ่ง

            # ---- วัดจุดจอด ----
            end = truth.still_pose(settle=1.0)
            odom1 = log.get()[0]
            amcl = log.fresh_amcl()

            rx, ry, ryaw = end
            # odom: การเคลื่อนที่ในกรอบหุ่นตอนเริ่ม ต่อจากจุดเริ่มจริง
            o = compose(st, compose(inverse(odom0), odom1)) if odom0 and odom1 else (nan, nan, nan)
            tfx, tfy = to_tape(tf_pose[:2], origin, tape) if tf_pose else (nan, nan)
            tfyaw = wrap(tf_pose[2] - phi_u - tape) if tf_pose else nan
            ax_, ay_ = to_tape(amcl[:2], origin, tape)
            ayaw = wrap(amcl[2] - phi_u - tape)
            moved_ok = int(rx - st[0] > 0.5 * d)
            odom_pts = [(t[2], t[3]) for t in traj if t[1] == 'odom']
            path_len = sum(math.dist(a, b) for a, b in zip(odom_pts, odom_pts[1:]))
            max_vx = max((abs(t[5]) for t in traj if t[1] == 'odom'), default=0.0)

            row = {
                'trial': name, 'condition': args.condition, 'order': order, 'dist': d, 'rep': rep,
                'result': result, 'moved_ok': moved_ok, 'duration_s': round(duration, 2),
                'recoveries': recoveries, 'amcl_updates': amcl_updates, 'battery_v': round(batt, 2),
                'scan_fixed': int(args.scan_fixed),
                'start_x': r4(st[0]), 'start_y': r4(st[1]), 'start_yaw_deg': round(math.degrees(st[2]), 2),
                'tape_yaw_deg': round(math.degrees(tape), 2), 'origin_x': r4(origin[0]), 'origin_y': r4(origin[1]),
                'goal_x': r4(gx), 'goal_y': r4(gy), 'init_err': r4(init_err),
                'real_x': r4(rx), 'real_y': r4(ry), 'real_yaw_deg': round(math.degrees(ryaw), 2),
                'tf_x': r4(tfx), 'tf_y': r4(tfy), 'tf_yaw_deg': round(math.degrees(tfyaw), 2),
                'amcl_x': r4(ax_), 'amcl_y': r4(ay_), 'amcl_yaw_deg': round(math.degrees(ayaw), 2),
                'odom_x': r4(o[0]), 'odom_y': r4(o[1]), 'odom_yaw_deg': round(math.degrees(o[2]), 2),
                'e_stop': r4(math.hypot(rx - d, ry)), 'e_stop_long': r4(rx - d), 'e_stop_lat': r4(ry),
                'e_tf': r4(math.hypot(tfx - rx, tfy - ry)),
                'e_amcl': r4(math.hypot(ax_ - rx, ay_ - ry)),
                'e_odom': r4(math.hypot(o[0] - rx, o[1] - ry)),
                'e_odom_long': r4(o[0] - rx), 'e_odom_lat': r4(o[1] - ry),
                'odom_scale': r4((o[0] - st[0]) / (rx - st[0])) if abs(rx - st[0]) > 0.05 else nan,
                'e_yaw_real_deg': round(math.degrees(ryaw), 2),
                'e_yaw_tf_deg': round(math.degrees(wrap(tfyaw - ryaw)), 2),
                'e_yaw_odom_deg': round(math.degrees(wrap(o[2] - ryaw)), 2),
                'home_err': r4(home_err), 'odom_path_len': r4(path_len), 'max_vx': round(max_vx, 3),
            }
            new = not os.path.exists(csv_path)
            with open(csv_path, 'a', newline='') as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                if new:
                    w.writeheader()
                w.writerow(row)
            with open(os.path.join(out_dir, 'traj', name + '.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'src', 'x', 'y', 'yaw', 'vx', 'wz'])
                w.writerows(traj)
            print(f'  ผล {result} {duration:.1f}s rec {recoveries} AMCL อัปเดต {amcl_updates}  แบต {batt:.2f} V', flush=True)
            print(f'  จอดจริง ({rx * 100:.1f}, {ry * 100:+.1f}) cm {math.degrees(ryaw):+.1f}°'
                  f' -> ห่างเป้า {row["e_stop"] * 100:.1f} cm (ยาว {row["e_stop_long"] * 100:+.1f}, ข้าง {ry * 100:+.1f})'
                  f'  | Nav2 เชื่อคลาด {row["e_tf"] * 100:.1f}  odom คลาด {row["e_odom"] * 100:.1f} cm', flush=True)
            if args.tape_wait:
                tape_check(out_dir, row, mark, end, st, o, (tfx, tfy, tfyaw), (ax_, ay_, ayaw))
            since_rest += 1
        if not args.manual:
            print('\nกลับจุดเริ่ม', flush=True)
            go_home(log, truth)
    except SafetyStop as e:
        print(f'\nหยุด: {e}')
    except KeyboardInterrupt:
        print('\nหยุดกลางคัน')
    finally:
        try:
            if not nav.isTaskComplete():
                nav.cancelTask()
        except Exception:
            pass
        stop_robot(log, 1.5)   # ส่งซ้ำ: WiFi ทำข้อความหายได้ และหุ่นค้างคำสั่งสุดท้ายไว้
        print('\nจบ', flush=True)
        ex.shutdown()
        nav.destroy_node()
        log.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

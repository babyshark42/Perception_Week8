#!/usr/bin/env python3
"""เก็บ bag สำหรับปรับเทียบ odometry และมุม LiDAR แล้ววิเคราะห์ด้วย tools/bag_calibrate.py

ลำดับอัตโนมัติ (วางหุ่นที่จุดเริ่มครั้งเดียว หุ่นวิ่งไป-กลับเอง):
  ทุกความเร็วใน SPEEDS x REPEATS:  นิ่ง 3 s -> เดินหน้า TARGET m (ตาม odom) -> นิ่ง 3 s -> ถอยกลับ -> นิ่ง 3 s
  หมุนอยู่กับที่ +170° / -170° x 2  (นิ่ง 3 s คั่น)
ช่วงนิ่งทำให้ได้ scan ที่ไม่บิดเบี้ยว ใช้ ICP เทียบก่อน/หลังแต่ละช่วงได้แม่น

ก่อนรัน: เปิด bringup บนหุ่น, อย่าสั่งเป้าหมาย Nav2 ระหว่างนี้ (controller จะแย่ง /cmd_vel), ข้างหน้าต้องว่าง > TARGET + 25 cm
ความปลอดภัย: เช็กระยะว่างก่อนเดินหน้าทุกครั้ง หยุดเองถ้า LiDAR เห็นของใกล้กว่า 18 cm (หน้า) / 8 cm (หลัง)
             หมุนเฉพาะเมื่อรอบตัวว่าง > 22 cm (ไปหมุนกลางห้อง แล้วถอยกลับ)
ใช้:  python3 calibrate_odom.py                     (เก็บ bag + วิเคราะห์)
      python3 calibrate_odom.py --tape              (ถามระยะตลับเมตรหลังเดินหน้าแต่ละครั้ง ไว้ตรวจ ICP)
      python3 calibrate_odom.py --write             (เขียน robot/burger_after.yaml จากผล real/odom)
      python3 calibrate_odom.py --analyze-only results/calib_bags/<ชื่อ>   (วิเคราะห์ bag ที่เก็บไว้แล้ว)
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import threading
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
SPEEDS = [0.05, 0.10, 0.15]   # m/s
REPEATS = 2
TARGET = 1.0                  # m (ตาม odom)
ROT_DEG = 170.0               # ไม่ใช้ 180 กันมุม wrap กำกวม
ROT_W = 0.5                   # rad/s
STILL_S = 3.0
STOP_FRONT = 0.18             # LiDAR เห็นของใกล้กว่านี้ในทิศที่วิ่ง -> หยุด (LiDAR อยู่หลังกึ่งกลางหุ่น 3 cm)
STOP_BACK = 0.08              # ถอย: จุดเริ่มเดิมกำแพงหลังห่าง LiDAR ~0.11 m จึงตั้งต่ำกว่า
FRONT_MARGIN = 0.25           # ก่อนเดินหน้า ต้องว่างข้างหน้า > TARGET + ค่านี้
ROT_MID = 0.40                # ไปหมุนกลางห้อง: เดินหน้าเท่านี้ก่อนหมุน แล้วถอยกลับ
ROT_CLEAR = 0.22              # หมุนได้เมื่อรอบตัวว่างกว่านี้


class SafetyStop(Exception):
    pass
TOPICS = ['/odom', '/scan', '/imu', '/cmd_vel', '/tf_static', '/battery_state', '/joint_states']


def analyze(bag, write, r_old, extra=()):
    out = os.path.join(ROOT, 'results', 'calib', os.path.basename(bag.rstrip('/')))
    if subprocess.run(['python3', '-s', os.path.join(ROOT, 'tools', 'bag_calibrate.py'), bag,
                       '--out', out, *extra]).returncode != 0:
        print('วิเคราะห์ไม่ได้ (ไม่มีช่วงที่หุ่นวิ่ง?)')
        return
    with open(os.path.join(out, 'calib.json')) as f:
        res = json.load(f)
    if write and res.get('real_over_odom'):
        write_after(res['real_over_odom'], r_old, res.get('n_straight', 0))


def write_after(k, r_old, n):
    r_new = r_old * k
    src = os.path.join(ROOT, 'robot', 'burger_before.yaml')
    dst = os.path.join(ROOT, 'robot', 'burger_after.yaml')
    with open(src) as f:
        txt = ''.join(ln for ln in f if not ln.startswith('#'))  # ตัดหัวไฟล์ "ค่าเดิม" ออก
    txt = re.sub(r'(radius:\s*)[0-9.]+', rf'\g<1>{r_new:.5f}', txt, count=1)
    txt = f'# สร้างโดย calibrate_odom.py: real/odom = {k:.4f} (n={n}), radius {r_old} -> {r_new:.5f}\n' + txt
    with open(dst, 'w') as f:
        f.write(txt)
    print(f'รัศมีล้อ {r_old} -> {r_new:.5f} m  เขียน {dst} แล้ว')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', type=float, default=TARGET)
    ap.add_argument('--repeats', type=int, default=REPEATS)
    ap.add_argument('--tape', action='store_true', help='ถามระยะตลับเมตรหลังเดินหน้าแต่ละครั้ง')
    ap.add_argument('--no-rotate', action='store_true')
    ap.add_argument('--write', action='store_true', help='เขียน robot/burger_after.yaml จากผล')
    ap.add_argument('--r-old', type=float, default=0.033, help='รัศมีล้อที่หุ่นใช้อยู่ตอนนี้ (m)')
    ap.add_argument('--analyze-only', metavar='BAG')
    args = ap.parse_args()
    if args.analyze_only:
        analyze(args.analyze_only, args.write, args.r_old)
        return

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    class Driver(Node):
        def __init__(self):
            super().__init__('fra503_w8_odom_cal')
            self.lock = threading.Lock()
            self.odom = None
            self.yaw_u = 0.0      # yaw แบบไม่ wrap
            self.pub = None
            self.stamped = False
            self.scan = None
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)

        def clearance(self, center_deg, half_deg):
            """ระยะที่ใกล้ที่สุดใน sector (วัดจาก LiDAR) None = ยังไม่มี scan"""
            m = self.scan
            if m is None:
                return None
            c = math.radians(center_deg)
            best = float('inf')
            for i, r in enumerate(m.ranges):
                a = m.angle_min + i * m.angle_increment - c
                if abs(math.atan2(math.sin(a), math.cos(a))) <= math.radians(half_deg) and math.isfinite(r) and r > 0.05:
                    best = min(best, r)
            return best

        def corridor(self, forward=True, half_width=0.12):
            """ระยะถึงสิ่งกีดขวางในช่องทางวิ่งกว้าง 2*half_width (วัดจาก LiDAR ตามแกนหุ่น)"""
            m = self.scan
            if m is None:
                return None
            best = float('inf')
            for i, r in enumerate(m.ranges):
                if not (math.isfinite(r) and r > 0.05):
                    continue
                a = m.angle_min + i * m.angle_increment
                x, y = r * math.cos(a), r * math.sin(a)
                if abs(y) < half_width and (x > 0) == forward:
                    best = min(best, abs(x))
            return best

        def setup_pub(self):
            """หุ่นบางเวอร์ชันรับ /cmd_vel เป็น TwistStamped (enable_stamped_cmd_vel) — ดูจาก subscriber ของหุ่น"""
            types = {i.topic_type for i in self.get_subscriptions_info_by_topic('/cmd_vel')}
            if not types:
                return False
            self.stamped = 'geometry_msgs/msg/TwistStamped' in types
            self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, '/cmd_vel', 10)
            return True

        def on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self.lock:
                if self.odom is not None:
                    self.yaw_u += math.atan2(math.sin(yaw - self.odom[2]), math.cos(yaw - self.odom[2]))
                self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

        def state(self):
            with self.lock:
                return self.odom, self.yaw_u

        def send(self, v, w=0.0):
            if self.stamped:
                t = TwistStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'
                tw = t.twist
            else:
                t = tw = Twist()
            tw.linear.x = v
            tw.angular.z = w
            self.pub.publish(t)

        def still(self, s=STILL_S):
            t0 = time.time()
            while time.time() - t0 < s:
                self.send(0.0)
                time.sleep(0.05)

        def drive(self, dist, v):
            """วิ่งตรงจน odom บอกว่าได้ |dist| (dist < 0 = ถอย) ชะลอ 5 cm สุดท้าย"""
            p0, _ = self.state()
            sgn = 1.0 if dist > 0 else -1.0
            while True:
                p, _ = self.state()
                d = math.dist(p[:2], p0[:2])
                if d >= abs(dist):
                    break
                c = self.corridor(forward=sgn > 0)
                if c is not None and c < (STOP_FRONT if sgn > 0 else STOP_BACK):
                    self.still(0.5)
                    raise SafetyStop(f'สิ่งกีดขวางห่าง {c:.2f} m ({"หน้า" if sgn > 0 else "หลัง"}) หยุดฉุกเฉิน')
                self.send(sgn * (v if abs(dist) - d > 0.05 else min(v, 0.03)))
                time.sleep(0.02)
            self.still(0.3)
            return d

        def rotate(self, deg, w=ROT_W):
            _, y0 = self.state()
            target = math.radians(deg)
            sgn = 1.0 if deg > 0 else -1.0
            while True:
                _, y = self.state()
                left = abs(target) - abs(y - y0)
                if left <= 0:
                    break
                self.send(0.0, sgn * (w if left > 0.15 else 0.15))
                time.sleep(0.02)
            self.still(0.3)

    rclpy.init()
    node = Driver()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    while node.state()[0] is None or node.scan is None or not node.setup_pub():
        print('รอ /odom และ subscriber ของ /cmd_vel (เปิด bringup บนหุ่นหรือยัง? ปิด Nav2 หรือยัง?) ...')
        time.sleep(0.5)
    print('/cmd_vel ชนิด', 'TwistStamped' if node.stamped else 'Twist')
    print(f'ระยะว่างในช่องทางวิ่ง: หน้า {node.corridor(True):.2f}  หลัง {node.corridor(False):.2f} m'
          f'  | รอบตัวใกล้สุด {node.clearance(0, 180):.2f} m')

    plan = [(v, r) for r in range(1, args.repeats + 1) for v in SPEEDS]
    print(f'\nลำดับ: {len(plan)} รอบไป-กลับ {args.target:.2f} m (ความเร็ว {SPEEDS})'
          + ('' if args.no_rotate else f' + หมุน ±{ROT_DEG:.0f}° x 2'))
    input('วางหุ่นชิดเทปจุดเริ่ม หันไปตามแนวเทป ให้ห้องโล่ง (คนออกจากระยะ LiDAR) แล้วกด Enter ')

    os.makedirs(os.path.join(ROOT, 'results', 'calib_bags'), exist_ok=True)
    bag = os.path.join(ROOT, 'results', 'calib_bags', time.strftime('calib_%Y%m%d_%H%M%S'))
    rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag, *TOPICS],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)  # ให้ recorder subscribe ครบก่อน
    tape = []
    try:
        node.still()
        for i, (v, r) in enumerate(plan, 1):
            c = node.corridor(forward=True)
            if c is None or c < args.target + FRONT_MARGIN:
                raise SafetyStop(f'ข้างหน้าว่าง {c} m ไม่พอสำหรับ {args.target} m (ต้อง > {args.target + FRONT_MARGIN:.2f})')
            print(f'[{i}/{len(plan)}] v={v:.2f} เดินหน้า', flush=True)
            d = node.drive(args.target, v)
            node.still()
            if args.tape:
                s = input(f'  odom {d * 100:.1f} cm  ระยะจริงตามแนวเทป (cm, Enter=ข้าม): ').strip()
                if s:
                    tape.append((v, d, float(s) / 100.0))
                node.still(1.0)
            print(f'[{i}/{len(plan)}] v={v:.2f} ถอยกลับ', flush=True)
            node.drive(-args.target, v)
            node.still()
        if not args.no_rotate:
            print(f'ไปกลางห้อง {ROT_MID} m เพื่อหมุน', flush=True)
            node.drive(ROT_MID, 0.10)
            node.still()
            c = node.clearance(0.0, 180.0)
            if c is None or c < ROT_CLEAR:
                print(f'  รอบตัวว่างแค่ {c} m ไม่หมุน')
            else:
                for deg in (ROT_DEG, -ROT_DEG, ROT_DEG, -ROT_DEG):
                    print(f'หมุน {deg:+.0f}°', flush=True)
                    node.rotate(deg)
                    node.still()
            node.drive(-ROT_MID, 0.10)
            node.still()
    except SafetyStop as e:
        print(f'\nหยุด: {e}')
    except KeyboardInterrupt:
        print('\nหยุดกลางคัน')
    finally:
        node.still(0.5)
        rec.send_signal(signal.SIGINT)
        rec.wait(timeout=15)
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    print(f'\nบันทึก bag -> {bag}')
    if tape:
        with open(bag + '_tape.csv', 'w') as f:
            f.write('speed,odom_m,tape_m\n' + ''.join(f'{v},{d:.4f},{t:.4f}\n' for v, d, t in tape))
        k = sum(t for _, _, t in tape) / sum(d for _, d, _ in tape)
        print(f'ตลับเมตร: real/odom = {k:.4f} ({len(tape)} ครั้ง)')
    analyze(bag, args.write, args.r_old)


if __name__ == '__main__':
    main()

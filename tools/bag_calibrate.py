#!/usr/bin/python3 -s
"""ปรับเทียบ odometry จาก ros bag โดยใช้ LiDAR + กำแพงห้องเป็นค่าจริง (ไม่ต้องใช้ตลับเมตร)

หลักการ: แบ่ง bag เป็นช่วงที่หุ่นวิ่ง (ดูจาก /odom twist) แล้วเอา scan ตอนหยุดนิ่งก่อนวิ่ง
มาเทียบกับ scan ตอนหยุดนิ่งหลังวิ่งด้วย ICP (point-to-line) -> ได้การเคลื่อนที่จริง (dx, dy, dθ)
ในกรอบหุ่นตอนเริ่ม แล้วเทียบกับ odom ช่วงเดียวกัน
  - dx_real / dx_odom      -> ค่าปรับรัศมีล้อ  r_new = r_old * (real/odom)
  - dy_real, dθ_real       -> หุ่นวิ่งเฉ (ไปซ้าย/ขวาเท่าไร, หันไปกี่องศาต่อเมตร)
  - dθ_odom เทียบ dθ_real  -> odom (ล้อ + IMU) รู้ตัวไหมว่าเฉ
ระหว่างวิ่งยัง ICP ทุก scan เทียบกับ scan เริ่ม เพื่อพล็อตเส้นทางจริงเทียบ odom

ใช้:  ./tools/bag_calibrate.py ../Perception_Week7/bags/run_slow_straight_1 ../Perception_Week7/bags/run_slow_straight_2 ...
      ./tools/bag_calibrate.py ../Perception_Week7/bags/Squretest_all3 --labels ../Perception_Week7/results   (ตั้งชื่อช่วงตามรอบ HW7)
ผล:  results/bag_calib/segments.csv, summary.md, figs/<bag>_seg<k>.png
"""
import argparse
import csv
import glob
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R_OLD = 0.033
STILL_V, STILL_W = 0.002, 0.01      # odom twist ต่ำกว่านี้ = หยุดนิ่ง
MIN_STILL_S = 0.6                   # ต้องนิ่งอย่างน้อยกี่วินาทีถึงนับเป็นช่วงหยุด
MIN_DISP = 0.05                     # ช่วงที่เคลื่อนน้อยกว่านี้ไม่นับ (m)
STRAIGHT_DEG = 10.0                 # ช่วงที่ odom หมุนเกินนี้ ไม่ใช้คำนวณ scale
DEFAULT_SCAN_X = -0.032             # base_footprint -> base_scan ของ Burger (ถ้า bag ไม่มี tf_static)


# ---------------- 2D transform helpers: (x, y, th) ----------------
def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------- อ่าน bag ----------------
def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in ('/odom', '/scan', '/tf_static', '/cmd_vel') if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    odom, scans, cmd = [], [], []
    scan_x = None
    while r.has_next():
        topic, raw, t_ns = r.read_next()
        m = deserialize_message(raw, get_message(types[topic]))
        t = t_ns * 1e-9
        if topic == '/odom':
            p, tw = m.pose.pose, m.twist.twist
            odom.append((t, p.position.x, p.position.y, yaw(p.orientation), tw.linear.x, tw.angular.z))
        elif topic == '/scan':
            a = m.angle_min + m.angle_increment * np.arange(len(m.ranges))
            rr = np.asarray(m.ranges, dtype=float)
            scans.append((t, a, rr, m.range_min, m.range_max))
        elif topic == '/cmd_vel':
            cmd.append((t, m.linear.x, m.angular.z))
        elif topic == '/tf_static' and scan_x is None:
            scan_x = static_scan_offset(m)
    return np.array(odom), scans, np.array(cmd) if cmd else np.zeros((0, 3)), scan_x


def static_scan_offset(msg):
    """x ของ base_scan ในกรอบ base_footprint (รวม base_footprint->base_link->base_scan)"""
    tf = {(t.header.frame_id, t.child_frame_id): t.transform.translation.x for t in msg.transforms}
    x = tf.get(('base_link', 'base_scan'))
    if x is None:
        return None
    return x + tf.get(('base_footprint', 'base_link'), 0.0)


# ---------------- ICP point-to-line ----------------
def scan_xy(a, rr, rmin, rmax):
    ok = np.isfinite(rr) & (rr > max(rmin, 0.12)) & (rr < rmax)
    return np.c_[rr[ok] * np.cos(a[ok]), rr[ok] * np.sin(a[ok])]


def median_scan(group):
    """รวมหลาย scan ตอนนิ่ง: median ต่อ beam ลด noise"""
    a = group[0][1]
    rr = np.stack([g[2] for g in group])
    rr = np.where(np.isfinite(rr) & (rr > 0), rr, np.nan)
    with np.errstate(all='ignore'):
        med = np.nanmedian(rr, axis=0)
    return a, np.nan_to_num(med, nan=0.0), group[0][3], group[0][4]


class Reference:
    def __init__(self, pts):
        self.p = pts
        self.tree = cKDTree(pts)
        _, idx = self.tree.query(pts, k=min(7, len(pts)))
        n = np.zeros_like(pts)
        self.good = np.zeros(len(pts), bool)
        for i, nb in enumerate(idx):
            q = pts[nb] - pts[nb].mean(0)
            w, v = np.linalg.eigh(q.T @ q)
            n[i] = v[:, 0]
            self.good[i] = w[0] < 0.1 * w[1]   # ต้องเป็นเส้นตรง (กำแพง) ถึงใช้ได้
        self.n = n


def icp(ref, src, guess, iters=60):
    x = np.array(guess, float)
    for it in range(iters):
        max_d = 0.25 if it < 10 else (0.10 if it < 25 else 0.05)
        c, s = math.cos(x[2]), math.sin(x[2])
        q = src @ np.array([[c, s], [-s, c]]) + x[:2]
        d, j = ref.tree.query(q)
        ok = (d < max_d) & ref.good[j]
        if ok.sum() < 30:
            return None, 0
        p, n, qs, s0 = ref.p[j[ok]], ref.n[j[ok]], q[ok], src[ok]
        res = np.einsum('ij,ij->i', n, qs - p)
        dq = np.c_[-s * s0[:, 0] - c * s0[:, 1], c * s0[:, 0] - s * s0[:, 1]]  # d(Rq)/dθ
        J = np.c_[n[:, 0], n[:, 1], np.einsum('ij,ij->i', n, dq)]
        w = 1.0 / np.maximum(1.0, np.abs(res) / 0.01)      # Huber
        H = J.T @ (J * w[:, None])
        if np.linalg.cond(H) > 1e8:
            return None, 0
        dx = -np.linalg.solve(H, J.T @ (w * res))
        x += dx
        if np.abs(dx).max() < 1e-6 and it > 25:
            break
    return (x[0], x[1], wrap(x[2])), int(ok.sum())


# ---------------- แบ่งช่วง ----------------
def segments(odom):
    """คืน [(a0, a1, b0, b1)]: ช่วงนิ่งก่อน [a0,a1] -> วิ่ง -> ช่วงนิ่งหลัง [b0,b1]
    ถ้า bag เริ่ม/จบตอนหุ่นยังวิ่ง ใช้ต้น/ท้าย bag เป็นช่วงอ้างอิง (a0 == a1 หรือ b0 == b1)"""
    still = (np.abs(odom[:, 4]) < STILL_V) & (np.abs(odom[:, 5]) < STILL_W)
    t = odom[:, 0]
    runs, i = [], 0
    while i < len(still):      # หา run ของ still ที่ยาวพอ
        if still[i]:
            j = i
            while j + 1 < len(still) and still[j + 1]:
                j += 1
            if t[j] - t[i] >= MIN_STILL_S:
                runs.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    if not runs or runs[0][0] > t[0] + 0.5:
        runs.insert(0, (t[0], t[0]))
    if runs[-1][1] < t[-1] - 0.5:
        runs.append((t[-1], t[-1]))
    return [(a0, a1, b0, b1) for (a0, a1), (b0, b1) in zip(runs, runs[1:])]


def pick_scans(scans, st, lo, hi, near):
    """scan ในช่วงนิ่ง [lo, hi] ถ้าไม่มี (หุ่นกำลังวิ่ง) ใช้ scan เดียวที่ใกล้เวลา near ที่สุด"""
    idx = np.where((st >= lo) & (st <= hi))[0]
    if len(idx) == 0:
        idx = [int(np.argmin(np.abs(st - near)))]
    group = [scans[i] for i in idx]
    return group, float(np.mean([g[0] for g in group]))


def odom_at(odom, t):
    k = int(np.clip(np.searchsorted(odom[:, 0], t), 1, len(odom) - 1))
    a, b = odom[k - 1], odom[k]
    f = 0.0 if b[0] == a[0] else np.clip((t - a[0]) / (b[0] - a[0]), 0, 1)
    return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), wrap(a[3] + f * wrap(b[3] - a[3])))


def load_labels(results_dir):
    out = []
    if not results_dir:
        return out
    for f in glob.glob(os.path.join(results_dir, 'traj', '*.csv')):
        with open(f) as fh:
            ts = [float(r['t']) for r in csv.DictReader(fh)]
        if ts:
            out.append((os.path.basename(f)[:-4], ts[0], ts[-1]))
    return out


def label_for(labels, t0, t1):
    best = max(labels, key=lambda l: min(t1, l[2]) - max(t0, l[1]), default=None)
    if best and min(t1, best[2]) - max(t0, best[1]) > 0:
        return best[0]
    return ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--labels', help='โฟลเดอร์ results ของ HW7 (มี traj/*.csv) ใช้ตั้งชื่อช่วงตามรอบ')
    ap.add_argument('--out', default=os.path.join(ROOT, 'results', 'bag_calib'))
    ap.add_argument('--no-track', action='store_true', help='ไม่ ICP ระหว่างวิ่ง (เร็วขึ้น ไม่มีกราฟเส้นทาง)')
    ap.add_argument('--scan-yaw', type=float, default=0.0,
                    help='มุม LiDAR เทียบแกนหุ่น (deg) ที่จะชดเชย — ใช้ตรวจว่าค่าที่ประมาณได้ทำให้ dy ~ 0')
    args = ap.parse_args()
    os.makedirs(os.path.join(args.out, 'figs'), exist_ok=True)
    labels = load_labels(args.labels)
    real_by_trial = {}
    if args.labels and os.path.exists(os.path.join(args.labels, 'trials.csv')):
        with open(os.path.join(args.labels, 'trials.csv')) as f:
            for r in csv.DictReader(f):
                try:
                    if not math.isnan(float(r['real_x'])):
                        real_by_trial[r['trial']] = float(r['real_x']) - (float(r['goal_x']) - float(r['dist']))
                except (KeyError, ValueError):
                    pass

    rows = []
    for bag in args.bags:
        name = os.path.basename(bag.rstrip('/'))
        print(f'อ่าน {name} ...', flush=True)
        odom, scans, cmd, scan_x = read_bag(bag)
        scan_x = DEFAULT_SCAN_X if scan_x is None else scan_x
        X = (scan_x, 0.0, math.radians(args.scan_yaw))  # base_footprint -> base_scan
        st = np.array([s[0] for s in scans])
        for k, (a0, a1, b0, b1) in enumerate(segments(odom), 1):
            # scan ตอนนิ่ง: ก่อน = 1 s สุดท้ายก่อนออกตัว, หลัง = หลังหยุด 0.3 s ไปอีก 1 s
            before, tb = pick_scans(scans, st, max(a0, a1 - 1.0), a1, a1)
            after, ta = pick_scans(scans, st, min(b0 + 0.3, b1), min(b1, b0 + 1.3), b0)
            o0, o1 = odom_at(odom, tb), odom_at(odom, ta)
            od = compose(inverse(o0), o1)          # odom delta ในกรอบหุ่นตอนเริ่ม
            rotate = math.hypot(od[0], od[1]) < MIN_DISP
            if rotate and abs(math.degrees(od[2])) < 20.0:
                continue
            ref = Reference(scan_xy(*median_scan(before)))
            src = scan_xy(*median_scan(after))
            # odom guess -> กรอบ scanner:  S = X^-1 * D * X
            guess = compose(compose(inverse(X), od), X)
            sd, npts = icp(ref, src, guess)
            if sd is None:
                print(f'  seg {k}: ICP ไม่ลู่เข้า ข้าม')
                continue
            real = compose(compose(X, sd), inverse(X))
            real_th = od[2] + wrap(real[2] - od[2])   # ไม่ wrap แยกจาก odom (หมุน ~180°)
            back = od[0] < 0
            lab = label_for(labels, tb, ta)
            win = cmd[(cmd[:, 0] >= tb) & (cmd[:, 0] <= ta)] if len(cmd) else cmd
            row = {
                'bag': name, 'seg': k, 'trial': lab, 't_start': round(tb, 2), 'duration_s': round(ta - tb, 2),
                'cmd_v_max': round(float(win[:, 1].max()), 3) if len(win) else '',
                'kind': 'rotate' if rotate else ('back' if back else 'fwd'),
                'cmd_w_absmax': round(float(np.abs(win[:, 2]).max()), 3) if len(win) else '',
                'odom_dx': round(od[0], 4), 'odom_dy': round(od[1], 4), 'odom_dth_deg': round(math.degrees(od[2]), 2),
                'real_dx': round(real[0], 4), 'real_dy': round(real[1], 4),
                'real_dth_deg': round(math.degrees(real[2]), 2), 'icp_points': npts,
                'scale_real_over_odom': round(real[0] / od[0], 4) if abs(od[0]) > MIN_DISP else '',
                # ใช้ได้ = เดินหน้า, เกือบตรง, odom กับ LiDAR หมุนเท่ากัน (ไม่ใช่ตอนยกหุ่นด้วยมือ)
                'straight': int(not rotate and abs(math.degrees(od[2])) < STRAIGHT_DEG
                                and abs(math.degrees(wrap(real[2] - od[2]))) < 5.0),
                'rot_real_over_odom': round(real_th / od[2], 4) if rotate else '',
                # open-loop = สั่ง cmd_vel ตรง ๆ w = 0 (ไม่ใช่ Nav2) ใช้ประมาณมุม LiDAR และการเฉ
                'open_loop': int(len(win) > 0 and float(np.abs(win[:, 2]).max()) < 1e-6),
                # ทิศที่เคลื่อนจริงเทียบแกนหุ่น (หักครึ่งหนึ่งของมุมที่หมุน = ทิศคอร์ดของส่วนโค้ง)
                'motion_dir_deg': '' if rotate else round(math.degrees(
                    (math.atan2(-real[1], -real[0]) if back else math.atan2(real[1], real[0])) - real[2] / 2), 2),
                'veer_deg_per_m': round(math.degrees(real[2]) / real[0], 2) if abs(real[0]) > MIN_DISP else '',
                'lat_cm_per_m': round(real[1] / real[0] * 100, 2) if abs(real[0]) > MIN_DISP else '',
                'tape_dx': round(real_by_trial[lab], 4) if lab in real_by_trial else '',
            }
            rows.append(row)
            print(f'  seg {k} {lab}: odom dx {od[0] * 100:6.1f} cm  real dx {real[0] * 100:6.1f} cm'
                  f'  real dy {real[1] * 100:+5.1f} cm  dθ real {math.degrees(real[2]):+5.1f}°'
                  f' odom {math.degrees(od[2]):+5.1f}°' + (f'  ตลับเมตร {row["tape_dx"] * 100:.1f}' if row['tape_dx'] != '' else ''))

            if args.no_track or rotate:
                continue
            # เส้นทางระหว่างวิ่ง: ICP ทุก scan เทียบ scan เริ่ม (เดา = odom ณ เวลานั้น)
            tr_o, tr_r = [], []
            for i in np.where((st > tb) & (st <= ta))[0]:
                t = scans[i][0]
                oi = compose(inverse(o0), odom_at(odom, t))
                g = compose(compose(inverse(X), oi), X)
                s_i, _ = icp(ref, scan_xy(*scans[i][1:]), g, iters=30)
                if s_i is None:
                    continue
                tr_o.append(oi)
                tr_r.append(compose(compose(X, s_i), inverse(X)))
            if len(tr_r) < 3:
                continue
            tr_o, tr_r = np.array(tr_o), np.array(tr_r)
            fig, ax = plt.subplots(1, 2, figsize=(10, 4))
            ax[0].plot(tr_o[:, 0] * 100, tr_o[:, 1] * 100, '.-', label='odom')
            ax[0].plot(tr_r[:, 0] * 100, tr_r[:, 1] * 100, '.-', label='LiDAR (ICP)')
            ax[0].set(xlabel='forward (cm)', ylabel='left (+) (cm)', title=f'{name} seg {k} {lab}')
            ax[1].plot(tr_o[:, 0] * 100, np.degrees(tr_o[:, 2]), '.-', label='odom')
            ax[1].plot(tr_r[:, 0] * 100, np.degrees(tr_r[:, 2]), '.-', label='LiDAR (ICP)')
            ax[1].set(xlabel='forward (cm)', ylabel='heading (deg)')
            for a in ax:
                a.grid(alpha=0.3)
                a.legend()
            fig.tight_layout()
            fig.savefig(os.path.join(args.out, 'figs', f'{name}_seg{k}.png'), dpi=120)
            plt.close(fig)

    if not rows:
        raise SystemExit('ไม่พบช่วงที่ใช้ได้')
    with open(os.path.join(args.out, 'segments.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    st_rows = [r for r in rows if r['straight']]
    calib = {'n_segments': len(rows), 'n_straight': len(st_rows), 'scan_yaw_applied_deg': args.scan_yaw}
    ol_rows = [r for r in st_rows if r['open_loop']]
    md = ['# ปรับเทียบ odometry จาก bag (LiDAR ICP)', '',
          f'ช่วงทั้งหมด {len(rows)}, ใช้ได้ (เดินหน้า เกือบตรง ไม่ได้ยกหุ่น) {len(st_rows)},'
          f' ในนั้นเป็น open-loop (cmd_vel w = 0) {len(ol_rows)} ช่วง', '']
    if st_rows:
        k = np.array([r['scale_real_over_odom'] for r in st_rows])
        dist = np.abs([r['real_dx'] for r in st_rows])
        kw = float((k * dist).sum() / dist.sum())            # ถ่วงด้วยระยะ (ช่วงยาวเชื่อได้กว่า)
        calib['real_over_odom'] = round(kw, 5)
        md += ['| ค่า | ผล |', '|---|---|',
               f'| real/odom (ถ่วงตามระยะ, {len(st_rows)} ช่วง) | {kw:.4f} (ช่วง {k.min():.3f}–{k.max():.3f},'
               f' SD {k.std(ddof=1) if len(k) > 1 else 0:.3f}) |',
               f'| รัศมีล้อที่สอดคล้อง | {R_OLD} × {kw:.4f} = {R_OLD * kw:.5f} m |']
        if ol_rows:
            d_ol = np.abs([r['real_dx'] for r in ol_rows])
            mdir = np.array([r['motion_dir_deg'] for r in ol_rows])
            for kind, label in (('fwd', 'เดินหน้า'), ('back', 'ถอยหลัง')):
                m = [r['motion_dir_deg'] for r in ol_rows if r['kind'] == kind]
                if m:
                    md.append(f'| ทิศที่เคลื่อนจริงเทียบแกน LiDAR ({label} {len(m)} ช่วง) | {np.mean(m):+.2f}° |')
            dur = np.array([r['duration_s'] for r in ol_rows])
            calib['scan_yaw_deg'] = round(float(args.scan_yaw - mdir.mean()), 3)
            calib['scan_yaw_sd_deg'] = round(float(mdir.std(ddof=1)), 3) if len(mdir) > 1 else 0.0
            md += [f'| ทิศที่เคลื่อนจริงเทียบแกน LiDAR (open-loop) | {mdir.mean():+.2f}° (SD {mdir.std(ddof=1) if len(mdir) > 1 else 0:.2f})'
                   f' -> ถ้าคงที่ทุกความเร็ว = LiDAR ติดเอียง **{args.scan_yaw - mdir.mean():+.2f}°** (+ = หันซ้าย) |',
                   f'| หุ่นหันจริงระหว่างวิ่ง (open-loop) | {sum(r["real_dth_deg"] for r in ol_rows) / d_ol.sum():+.2f} °/m |',
                   f'| odom (ล้อ + IMU) หันระหว่างวิ่ง | {sum(r["odom_dth_deg"] for r in ol_rows) / dur.sum():+.4f} °/s'
                   f' (ถ้าคงที่ต่อวินาที ไม่ใช่ต่อเมตร = gyro bias) |']
    # เฉพาะหมุนอยู่กับที่ที่สั่งเอง (cmd v = 0) — ช่วงหมุนใน bag ของ Nav2 มักเป็นตอนยกหุ่น
    rot = [r for r in rows if r['kind'] == 'rotate' and r['cmd_v_max'] != '' and abs(r['cmd_v_max']) < 1e-6]
    if rot:
        q = np.array([r['rot_real_over_odom'] for r in rot])
        calib['rot_real_over_odom'] = round(float(q.mean()), 5)
        md.append(f'| หมุนจริง / odom หมุน ({len(rot)} ครั้ง) | {q.mean():.4f} (SD {q.std(ddof=1) if len(q) > 1 else 0:.4f})'
                  f' — odom ของ TB3 ใช้ IMU หาทิศ ค่านี้จึงตรวจ gyro ไม่ใช่ระยะห่างล้อ |')
    md.append('')
    tape = [r for r in rows if r['tape_dx'] != '']
    if tape:
        md += ['**ตรวจ ICP กับตลับเมตร (HW7)**', '', '| รอบ | ตลับเมตร (cm) | LiDAR ICP (cm) | odom (cm) |',
               '|---|---|---|---|']
        md += [f'| {r["trial"]} | {r["tape_dx"] * 100:.1f} | {r["real_dx"] * 100:.1f} | {r["odom_dx"] * 100:.1f} |'
               for r in tape]
        md.append('')
    md += ['| bag | seg | รอบ | cmd v | odom dx (cm) | จริง dx (cm) | real/odom | จริง dy (cm) | dθ จริง (°) | dθ odom (°) |',
           '|---|---|---|---|---|---|---|---|---|---|']
    md += [f'| {r["bag"]} | {r["seg"]} | {r["trial"]} | {r["cmd_v_max"]} | {r["odom_dx"] * 100:.1f} | '
           f'{r["real_dx"] * 100:.1f} | {r["scale_real_over_odom"]} | {r["real_dy"] * 100:+.1f} | '
           f'{r["real_dth_deg"]:+.1f} | {r["odom_dth_deg"]:+.1f} |' for r in rows]
    with open(os.path.join(args.out, 'calib.json'), 'w') as f:
        json.dump(calib, f, indent=1)
    with open(os.path.join(args.out, 'summary.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    print('\n' + '\n'.join(md[:11]))
    print(f'\nเขียน {args.out}/segments.csv, summary.md, figs/')


if __name__ == '__main__':
    main()

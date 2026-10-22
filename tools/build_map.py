#!/usr/bin/python3 -s
"""สร้างแผนที่ห้องใหม่จาก scan ตอนหุ่นนิ่งใน bag โดยใช้ตำแหน่งจาก ICP (กรอบเทป) — ใช้แทน square_room ที่สั้นกว่าจริง ~10 cm

กรอบแผนที่ = กรอบเทป: (0,0) = จุดเริ่มบนเทป, แกน x = ตามเทป (ตรงกับทิศหุ่นจริง ไม่ใช่ทิศ LiDAR)
จุดของ scan แปลงด้วยมุม LiDAR จริง (SCAN_YAW_DEG) -> ต้องใช้คู่กับ /scan_fixed (scan_rotate.py) หรือ URDF ที่แก้แล้ว
ใช้:  ./tools/build_map.py results/calib_bags/calib_20261014_171621 --out maps/room_icp
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_calibrate import Reference, icp, scan_xy, median_scan, compose, inverse, read_bag, segments  # noqa: E402

SCAN_YAW_DEG = 1.8
SCAN_X = -0.032


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', required=True, help='ไม่ต้องใส่นามสกุล -> <out>.pgm + <out>.yaml')
    ap.add_argument('--res', type=float, default=0.025)
    ap.add_argument('--ref', default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                  'results', 'ref_scan.npz'))
    args = ap.parse_args()

    X = (SCAN_X, 0.0, math.radians(SCAN_YAW_DEG))
    z = np.load(args.ref)
    ref_scan = (z['a'], z['r'], float(z['rmin']), float(z['rmax']))
    ref = Reference(scan_xy(*ref_scan))
    odom, scans, _, _ = read_bag(args.bag)
    st = np.array([s[0] for s in scans])

    # ช่วงนิ่งทั้งหมด -> median scan -> ICP เทียบ ref (เดาจาก odom ต่อจากช่วงก่อน)
    stills = []
    for a0, a1, b0, b1 in segments(odom):
        stills.append((a0, a1))
    stills.append((segments(odom)[-1][2], segments(odom)[-1][3]))
    poses = []
    prev_pose, prev_odom = (0.0, 0.0, 0.0), None
    for a0, a1 in stills:
        idx = np.where((st >= a0 + 0.3) & (st <= a1))[0]
        if len(idx) < 3:
            continue
        group = [scans[i] for i in idx]
        k = int(np.clip(np.searchsorted(odom[:, 0], a1), 0, len(odom) - 1))
        o = tuple(odom[k, 1:4])
        guess = prev_pose if prev_odom is None else compose(prev_pose, compose(inverse(prev_odom), o))
        s, n = icp(ref, scan_xy(*median_scan(group)), compose(compose(inverse(X), guess), X), iters=80)
        if s is None:
            print(f'  ข้ามช่วงนิ่ง t={a1:.1f} (ICP ไม่ลู่เข้า)')
            continue
        body = compose(compose(X, s), inverse(X))
        prev_pose, prev_odom = body, o
        poses.append((body, median_scan(group)))
        print(f'  scan นิ่ง {len(group):2d} อัน  pose ({body[0] * 100:6.1f}, {body[1] * 100:+5.1f}) cm'
              f' {math.degrees(body[2]):+7.2f}°  ICP {n} จุด')

    # รวมจุดในกรอบเทป
    pts_all, sensors = [], []
    for body, (a, r, rmin, rmax) in poses:
        lidar = compose(body, X)
        p = scan_xy(a, r, rmin, rmax)
        c, s_ = math.cos(lidar[2]), math.sin(lidar[2])
        pts_all.append(np.c_[lidar[0] + c * p[:, 0] - s_ * p[:, 1], lidar[1] + s_ * p[:, 0] + c * p[:, 1]])
        sensors.append(lidar[:2])
    allp = np.vstack(pts_all)
    lo = allp.min(0) - 0.3
    hi = allp.max(0) + 0.3
    res = args.res
    W, H = int(np.ceil((hi[0] - lo[0]) / res)), int(np.ceil((hi[1] - lo[1]) / res))
    hits = np.zeros((H, W), int)
    free = np.zeros((H, W), int)

    def cell(x, y):
        return int((x - lo[0]) / res), int((y - lo[1]) / res)

    for (sx, sy), pts in zip(sensors, pts_all):
        c0 = cell(sx, sy)
        for x, y in pts:
            c1 = cell(x, y)
            n = max(abs(c1[0] - c0[0]), abs(c1[1] - c0[1]))
            for i in range(n):   # ray: ช่องระหว่าง LiDAR กับจุดที่โดน = ว่าง
                cx = c0[0] + (c1[0] - c0[0]) * i // n
                cy = c0[1] + (c1[1] - c0[1]) * i // n
                free[cy, cx] += 1
            hits[c1[1], c1[0]] += 1
    occ = hits >= 2
    img = np.full((H, W), 205, np.uint8)        # unknown
    img[(free > 0) & ~occ] = 254                # free
    img[occ] = 0
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out + '.pgm', 'wb') as f:
        f.write(f'P5\n# week8 build_map.py ICP map\n{W} {H}\n255\n'.encode())
        f.write(np.flipud(img).tobytes())        # แถวบนสุดของ pgm = y มากสุด
    with open(args.out + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(args.out)}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{lo[0]:.4f}, {lo[1]:.4f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    # ขนาดห้องตามแนว y=0 และ x=0.4
    row = occ[int((0 - lo[1]) / res)]
    xs = lo[0] + (np.where(row)[0] + 0.5) * res
    col = occ[:, int((0.4 - lo[0]) / res)]
    ys = lo[1] + (np.where(col)[0] + 0.5) * res
    print(f'{len(poses)} ตำแหน่ง, แผนที่ {W}x{H} ช่อง ({res} m) -> {args.out}.pgm/.yaml')
    print(f'กำแพงตามแนว y=0: x = {xs.min():.3f} .. {xs.max():.3f}  (ยาว {xs.max() - xs.min():.3f} m)')
    print(f'กำแพงตามแนว x=0.4: y = {ys.min():.3f} .. {ys.max():.3f}  (กว้าง {ys.max() - ys.min():.3f} m)')


if __name__ == '__main__':
    main()

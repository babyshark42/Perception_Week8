#!/usr/bin/python3 -s
"""วิเคราะห์ผล Assignment 8: error เทียบระยะทาง และเทียบก่อน/หลังแก้

อ่าน results/<condition>/trials.csv ทุกโฟลเดอร์ แล้วเขียน
  results/summary.csv     ค่าเฉลี่ย/SD ต่อ condition x ระยะ
  results/summary.md      ตารางพร้อมใส่รายงาน (ค่าเฉลี่ย, ความชัน error ต่อระยะ, เทียบก่อน/หลัง + t-test)
  results/figs/*.png      กราฟ
ใช้:  ./analyze.py                     (baseline = ก่อนแก้)
      ./analyze.py --before baseline --results results
หมายเหตุ: ต้องรันด้วย python3 -s (shebang ใส่ให้แล้ว) — numpy 2 ใน ~/.local ทำให้ matplotlib ของระบบพัง
"""
import argparse
import csv
import glob
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
NUM = ['dist', 'duration_s', 'recoveries', 'amcl_updates', 'battery_v', 'init_err',
       'real_x', 'real_y', 'goal_x', 'goal_y',
       'e_stop', 'e_stop_long', 'e_stop_lat', 'e_tf', 'e_amcl', 'e_odom',
       'e_odom_long', 'e_odom_lat', 'odom_scale', 'e_yaw_real_deg', 'e_yaw_odom_deg']
# ค่าที่สรุป: (คอลัมน์, ชื่อในตาราง, คูณ)  ระยะเป็น cm
METRICS = [('e_stop', 'จอดห่างเป้าจริง (cm)', 100), ('e_stop_long', 'เกิน(+)/ขาด(-) เป้า (cm)', 100),
           ('e_stop_lat', 'เยื้องซ้าย(+) (cm)', 100), ('e_tf', '|Nav2 เชื่อ - จริง| (cm)', 100),
           ('e_amcl', '|AMCL - จริง| (cm)', 100), ('e_odom', '|odom - จริง| (cm)', 100),
           ('e_odom_long', 'odom เกิน(+) แนวยาว (cm)', 100), ('odom_scale', 'odom/จริง', 1),
           ('e_yaw_real_deg', 'มุมจริงเบี้ยว (deg)', 1), ('recoveries', 'recovery (ครั้ง)', 1),
           ('amcl_updates', 'AMCL อัปเดต (ครั้ง)', 1), ('duration_s', 'เวลา (s)', 1)]
FIT = [('e_stop', 'stop error vs goal'), ('e_tf', 'Nav2 belief error'),
       ('e_amcl', 'AMCL error'), ('e_odom', 'odom error')]


def load(results):
    rows = []
    for path in sorted(glob.glob(os.path.join(results, '*', 'trials.csv'))):
        with open(path) as f:
            for r in csv.DictReader(f):
                if r['result'] == 'INVALID':
                    continue
                for k in NUM:
                    try:
                        r[k] = float(r[k])
                    except (KeyError, ValueError):
                        r[k] = math.nan
                rows.append(r)
    return rows


def vals(rows, key, cond, dist=None):
    v = [r[key] for r in rows if r['condition'] == cond and (dist is None or r['dist'] == dist)]
    return np.array([x for x in v if not math.isnan(x)])


def msd(a, scale=1, nd=1):
    if len(a) == 0:
        return '–'
    if len(a) == 1:
        return f'{a[0] * scale:.{nd}f}'
    return f'{a.mean() * scale:.{nd}f} ± {a.std(ddof=1) * scale:.{nd}f}'


def linfit(rows, key, cond):
    pts = [(r['dist'], r[key]) for r in rows if r['condition'] == cond and not math.isnan(r[key])]
    if len(pts) < 3 or len({p[0] for p in pts}) < 2:
        return None
    x, y = np.array(pts).T
    res = stats.linregress(x, y * 100)
    return res.slope, res.intercept, res.rvalue ** 2, res.pvalue


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default=os.path.join(ROOT, 'results'))
    ap.add_argument('--before', default='baseline', help='condition ที่เป็น "ก่อนแก้"')
    args = ap.parse_args()

    rows = load(args.results)
    if not rows:
        raise SystemExit(f'ไม่พบข้อมูลใน {args.results}/*/trials.csv')
    conds = sorted({r['condition'] for r in rows}, key=lambda c: (c != args.before, c))
    dists = sorted({r['dist'] for r in rows})
    figs = os.path.join(args.results, 'figs')
    os.makedirs(figs, exist_ok=True)

    # ---------- summary.csv ----------
    out = []
    for c in conds:
        for d in dists:
            sel = [r for r in rows if r['condition'] == c and r['dist'] == d]
            if not sel:
                continue
            row = {'condition': c, 'dist': d, 'n': len(sel),
                   'succeeded': sum(r['result'] == 'SUCCEEDED' for r in sel),
                   'measured': int(len(vals(rows, 'e_stop', c, d)))}
            for k, _, s in METRICS:
                a = vals(rows, k, c, d) * s
                row[f'{k}_mean'] = round(float(a.mean()), 3) if len(a) else ''
                row[f'{k}_sd'] = round(float(a.std(ddof=1)), 3) if len(a) > 1 else ''
            out.append(row)
    with open(os.path.join(args.results, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader()
        w.writerows(out)

    # ---------- summary.md ----------
    md = ['# สรุปผล Assignment 8', '', 'ค่าในตาราง = ค่าเฉลี่ย ± SD (ตำแหน่งจริงจาก LiDAR ICP, ไม่นับรอบ INVALID)', '']
    for c in conds:
        md += [f'## {c}', '', '| ค่า | ' + ' | '.join(f'{d:.2f} m' for d in dists) + ' |',
               '|---|' + '---|' * len(dists)]
        ok = []
        for d in dists:
            sel = [r for r in rows if r['condition'] == c and r['dist'] == d]
            ok.append(f'{sum(r["result"] == "SUCCEEDED" for r in sel)}/{len(sel)}')
        md.append('| สำเร็จ/ทั้งหมด | ' + ' | '.join(ok) + ' |')
        for k, label, s in METRICS:
            md.append(f'| {label} | ' + ' | '.join(msd(vals(rows, k, c, d), s, 3 if k == 'odom_scale' else 1) for d in dists) + ' |')
        md += ['', '**ความชันของ error ต่อระยะทาง (linear regression ทุกรอบ)**', '',
               '| error | ความชัน (cm/m) | จุดตัด (cm) | R² | p |', '|---|---|---|---|---|']
        for k, label in FIT:
            fit = linfit(rows, k, c)
            if fit:
                md.append(f'| {label} | {fit[0]:.1f} | {fit[1]:.1f} | {fit[2]:.2f} | {fit[3]:.3f} |')
        md.append('')
    after = [c for c in conds if c != args.before]
    if args.before in conds and after:
        md += [f'## เทียบก่อน ({args.before}) / หลังแก้ — จอดห่างเป้าจริง (cm)', '',
               '| ระยะ | ' + ' | '.join([args.before] + after) + ' | ' +
               ' | '.join(f'เปลี่ยน % ({c})' for c in after) + ' | ' +
               ' | '.join(f'p Welch ({c})' for c in after) + ' |',
               '|---|' + '---|' * (1 + 3 * len(after))]
        for d in dists:
            b = vals(rows, 'e_stop', args.before, d)
            cells = [msd(b, 100)] + [msd(vals(rows, 'e_stop', c, d), 100) for c in after]
            pct, pv = [], []
            for c in after:
                a = vals(rows, 'e_stop', c, d)
                pct.append(f'{(a.mean() - b.mean()) / b.mean() * 100:+.0f}%' if len(a) and len(b) else '–')
                pv.append(f'{stats.ttest_ind(a, b, equal_var=False).pvalue:.3f}'
                          if len(a) > 1 and len(b) > 1 else '–')
            md.append(f'| {d:.2f} m | ' + ' | '.join(cells + pct + pv) + ' |')
        md += ['', 'p < 0.05 = ต่างกันอย่างมีนัยสำคัญ (n น้อย ให้ดูประกอบกับกราฟ)', '']
    with open(os.path.join(args.results, 'summary.md'), 'w') as f:
        f.write('\n'.join(md))

    # ---------- กราฟ ----------
    rng = np.random.default_rng(0)
    for c in conds:
        # 1) error 4 แบบ เทียบระยะ
        fig, ax = plt.subplots(figsize=(7, 4.5))
        for i, (k, label) in enumerate(FIT):
            off = (i - 1.5) * 0.02
            for d in dists:
                a = vals(rows, k, c, d) * 100
                if len(a):
                    ax.scatter(d + off + rng.uniform(-0.005, 0.005, len(a)), a, s=14, alpha=0.4, color=f'C{i}')
            m = [(d, vals(rows, k, c, d).mean() * 100, vals(rows, k, c, d).std(ddof=1) * 100
                  if len(vals(rows, k, c, d)) > 1 else 0) for d in dists if len(vals(rows, k, c, d))]
            if m:
                x, y, e = np.array(m).T
                ax.errorbar(x + off, y, yerr=e, marker='o', capsize=3, color=f'C{i}', label=label)
        ax.set(xlabel='distance (m)', ylabel='error (cm)', title=f'Error vs distance [{c}]', xticks=dists)
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(figs, f'errors_vs_distance_{c}.png'), dpi=150)
        plt.close(fig)

        # 2) odom แยกแนวยาว / แนวข้าง
        fig, ax = plt.subplots(figsize=(7, 4.5))
        for i, (k, label) in enumerate([('e_odom_long', 'odom - real, longitudinal'),
                                        ('e_odom_lat', 'odom - real, lateral'),
                                        ('e_stop_long', 'real - goal, longitudinal'),
                                        ('e_stop_lat', 'real - goal, lateral')]):
            pts = [(r['dist'], r[k] * 100) for r in rows if r['condition'] == c and not math.isnan(r[k])]
            if not pts:
                continue
            x, y = np.array(pts).T
            ax.scatter(x + (i - 1.5) * 0.015, y, s=16, alpha=0.6, color=f'C{i}', label=label)
            if len(set(x)) > 1:
                p = np.polyfit(x, y, 1)
                xx = np.array([0, max(dists)])
                ax.plot(xx, np.polyval(p, xx), '--', color=f'C{i}', lw=1)
        ax.axhline(0, color='k', lw=0.8)
        ax.set(xlabel='distance (m)', ylabel='signed error (cm)', title=f'Error components [{c}]',
               xticks=dists)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(figs, f'error_components_{c}.png'), dpi=150)
        plt.close(fig)

        # 3) ตำแหน่งจอดจริงรอบเป้า (มุมมองจากด้านบน)
        fig, ax = plt.subplots(figsize=(5.5, 5))
        for i, d in enumerate(dists):
            sel = [r for r in rows if r['condition'] == c and r['dist'] == d and not math.isnan(r['e_stop'])]
            if sel:
                ax.scatter([r['e_stop_long'] * 100 for r in sel], [r['e_stop_lat'] * 100 for r in sel],
                           s=30, color=f'C{i}', label=f'{d:.2f} m')
        ax.add_patch(plt.Circle((0, 0), 5, fill=False, ls='--', color='gray'))
        ax.axhline(0, color='k', lw=0.6)
        ax.axvline(0, color='k', lw=0.6)
        ax.set(xlabel='overshoot (+) / short (-) (cm)', ylabel='left (+) / right (-) (cm)',
               title=f'Final position around goal [{c}]\n(dashed = 5 cm goal tolerance)', aspect='equal')
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(figs, f'final_positions_{c}.png'), dpi=150)
        plt.close(fig)

    # 4) AMCL อัปเดต และ 5) เทียบก่อน/หลัง
    for key, ylabel, scale, fname in [('amcl_updates', 'AMCL updates per run', 1, 'amcl_updates'),
                                      ('e_stop', 'stop error vs goal (cm)', 100, 'compare_e_stop'),
                                      ('e_odom', 'odom error (cm)', 100, 'compare_e_odom')]:
        if fname.startswith('compare') and len(conds) < 2:
            continue
        fig, ax = plt.subplots(figsize=(7, 4.5))
        for i, c in enumerate(conds):
            m = [(d, vals(rows, key, c, d).mean() * scale,
                  vals(rows, key, c, d).std(ddof=1) * scale if len(vals(rows, key, c, d)) > 1 else 0)
                 for d in dists if len(vals(rows, key, c, d))]
            if m:
                x, y, e = np.array(m).T
                ax.errorbar(x + (i - (len(conds) - 1) / 2) * 0.02, y, yerr=e, marker='o', capsize=3,
                            label=c + (' (before)' if c == args.before else ''))
        ax.set(xlabel='distance (m)', ylabel=ylabel, xticks=dists)
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(figs, f'{fname}.png'), dpi=150)
        plt.close(fig)

    print('\n'.join(md))
    print(f'\nเขียน {args.results}/summary.csv, summary.md, figs/')


if __name__ == '__main__':
    main()

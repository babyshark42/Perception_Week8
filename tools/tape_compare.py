#!/usr/bin/python3 -s
"""เทียบระยะที่ระบบเก็บ (ICP / odom / AMCL / Nav2) กับตลับเมตร ก่อนจูนและหลังจูน

อ่าน results/tape_check/<cond>/tape_check.csv -> results/tape_check/summary.md + tape_vs_sources.png + tape_stop_error.png
ตำแหน่งทุกค่าเทียบเครื่องหมายที่พื้น (mark.json) ตลับเมตรคือค่าจริง
"""
import csv
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results', 'tape_check')
CONDS = [('baseline', 'ก่อนจูน'), ('tuned', 'หลังจูน')]
SRCS = [('icp', 'ICP (ค่าจริงที่ใช้ในผล)'), ('odom', 'odom'), ('amcl', 'AMCL'), ('nav2', 'Nav2 เชื่อ')]
DY_BAD = {('tuned', 'd0.75_r2')}     # ผู้วัดแจ้งว่า dy รอบนี้วัดผิด -> ใช้แค่ dx


def load():
    mark = json.load(open(os.path.join(ROOT, 'mark.json')))
    data = {}
    for c, _ in CONDS:
        p = os.path.join(ROOT, c, 'tape_check.csv')
        if os.path.exists(p):
            data[c] = list(csv.DictReader(open(p)))
    return mark, data


def f(r, k):
    return float(r[k])


def main():
    mark, data = load()
    mx, my = mark['x'] * 100, mark['y'] * 100     # cm: เครื่องหมายเทียบจุดเริ่มอ้างอิง
    dists = sorted({float(r['dist']) for rows in data.values() for r in rows})
    md = ['# เทียบระยะที่ระบบเก็บ กับตลับเมตร (ก่อนจูน / หลังจูน)', '',
          'ทุกค่าเทียบเครื่องหมายที่พื้นตอนเริ่ม หน่วย cm, ตลับเมตร = ค่าจริง, ระยะละ 3 รอบ', '']

    # 1) ความคลาดของแต่ละแหล่งเทียบตลับเมตร
    md += ['## แต่ละแหล่งคลาดจากตลับเมตรเท่าไร (ค่าเฉลี่ยของ |คลาด|)', '',
           '| ชุด | ระยะ | ' + ' | '.join(lbl for _, lbl in SRCS) + ' |', '|---|---|' + '---|' * len(SRCS)]
    summ = {}
    for c, cl in CONDS:
        for d in dists + ['รวม']:
            rows = [r for r in data.get(c, []) if d == 'รวม' or float(r['dist']) == d]
            if not rows:
                continue
            cells = []
            for k, _ in SRCS:
                e = []
                for r in rows:
                    ex = f(r, f'{k}_dx_cm') - f(r, 'tape_dx_cm')
                    ey = 0.0 if (c, r['trial']) in DY_BAD else f(r, f'{k}_dy_cm') - f(r, 'tape_dy_cm')
                    e.append(math.hypot(ex, ey))
                cells.append(f'{np.mean(e):.1f}')
                summ[(c, d, k)] = np.mean(e)
            md.append(f'| {cl} | {d if d == "รวม" else f"{d:.2f} m"} | ' + ' | '.join(cells) + ' |')
    md.append('')

    # 2) หุ่นจอดห่างเป้าจริง (ตลับเมตร) เทียบที่ ICP วัด
    md += ['## หุ่นจอดห่างเป้าจริงเท่าไร (วัดด้วยตลับเมตร) เทียบกับ ICP', '',
           '| ระยะ | ก่อนจูน: ตลับเมตร | ก่อนจูน: ICP | หลังจูน: ตลับเมตร | หลังจูน: ICP |', '|---|---|---|---|---|']
    stop = {}
    for d in dists:
        cells = []
        for c, _ in CONDS:
            rows = [r for r in data.get(c, []) if float(r['dist']) == d]
            tape = [math.hypot(f(r, 'tape_dx_cm') + mx - d * 100, f(r, 'tape_dy_cm') + my) for r in rows]
            icp = [math.hypot(f(r, 'icp_dx_cm') + mx - d * 100, f(r, 'icp_dy_cm') + my) for r in rows]
            stop[(c, d)] = (tape, icp)
            cells += [f'{np.mean(tape):.1f} ± {np.std(tape, ddof=1):.1f}' if len(tape) > 1 else '–',
                      f'{np.mean(icp):.1f}' if icp else '–']
        md.append(f'| {d:.2f} m | ' + ' | '.join(cells) + ' |')
    md += ['', f'หมายเหตุ: รอบ {", ".join(t for _, t in DY_BAD)} (tuned) ผู้วัดแจ้งว่า dy วัดผิด จึงนับเฉพาะ dx ในตารางแรก', '']
    with open(os.path.join(ROOT, 'summary.md'), 'w') as fh:
        fh.write('\n'.join(md))
    print('\n'.join(md))

    # กราฟ 1: ความคลาดแต่ละแหล่งเทียบระยะ
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for ax, (c, cl) in zip(axes, CONDS):
        for i, (k, lbl) in enumerate(SRCS):
            y = [summ.get((c, d, k), np.nan) for d in dists]
            ax.plot(dists, y, 'o-', label=['ICP', 'odom', 'AMCL', 'Nav2 belief'][i])
        ax.set(title=f'{c} vs tape measure', xlabel='distance (m)', xticks=dists)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('|source - tape| (cm)')
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'tape_vs_sources.png'), dpi=140)
    plt.close(fig)

    # กราฟ 2: หุ่นจอดห่างเป้า (ตลับเมตร) ก่อน/หลัง
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    for i, (c, cl) in enumerate(CONDS):
        m = [np.mean(stop[(c, d)][0]) for d in dists]
        s = [np.std(stop[(c, d)][0], ddof=1) for d in dists]
        ax.errorbar(np.array(dists) + (i - 0.5) * 0.02, m, yerr=s, marker='o', capsize=3,
                    label=f'{c} (tape)')
    ax.set(xlabel='distance (m)', ylabel='stop error vs goal (cm)', title='Stop error measured by tape',
           xticks=dists)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, 'tape_stop_error.png'), dpi=140)
    plt.close(fig)
    print(f'\nเขียน {ROOT}/summary.md, tape_vs_sources.png, tape_stop_error.png')


if __name__ == '__main__':
    main()

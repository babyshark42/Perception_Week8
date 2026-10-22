# FRA503 (AMR) Assignment 8: ผลของระยะทางต่อความแม่นของ Nav2

TurtleBot3 Burger หุ่นจริง ห้องสี่เหลี่ยม (`maps/square_room.yaml`) ROS 2 Humble

- **ข้อ 1** ทำการทดลองจาก HW7 ใหม่ โดยเปลี่ยนแค่ระยะทาง (4 ระยะ) แล้ววิเคราะห์ความสัมพันธ์ เหตุ และเสนอวิธีแก้
- **ข้อ 2** ทดสอบวิธีแก้ด้วยขั้นตอนเดียวกับข้อ 1 ทุกอย่าง แล้วเทียบผลก่อนและหลังแก้

ต่อจาก [Perception_Week7](../Perception_Week7) (HW7: การทดลอง Nav2 ครั้งแรก) รายงานส่ง: `FRA503(AMR)_ Assignment 8_68401.docx`, สรุปภาษาไทย: `สรุป_Assignment8.md`

## โครงสร้างโฟลเดอร์

```
Perception_Week8/
├── start_nav.sh            เปิด Nav2 + RViz ตาม condition (baseline / amclfix / scanfix / tuned)
├── run_experiment.py       รันการทดลองอัตโนมัติ 4 ระยะ × 5 รอบ วัดตำแหน่งจริงด้วย LiDAR ICP
├── calibrate_odom.py       ปรับเทียบ odom / มุม LiDAR บนหุ่นจริง
├── analyze.py              สรุปผล + วาดกราฟ
├── params/                 Nav2 params แต่ละ condition
├── maps/                   square_room (แผนที่เดิมจาก HW7) + room_icp (แผนที่ใหม่)
├── robot/                  ไฟล์ที่ใช้บนหุ่น (burger.yaml ก่อน/หลัง, fix_scan_yaw.sh)
├── tools/                  scan_rotate, build_map, bag_calibrate, tape_compare, snapshot_config
├── results/                ผลการทดลอง (csv, log, config ที่ใช้จริง, figs/)
├── bags/                   ros bag ของชุด baseline / tuned (ไม่ขึ้น git)
└── *.rviz                  config RViz (ปิด LaserScan display กัน deadlock)
```

ใน git มีแค่สคริปต์ ค่าตั้ง แผนที่ ผล (csv, log, กราฟ) และรายงาน ส่วน ros bag ทุกไฟล์ (`*.db3` ใน `bags/` และ `results/calib_bags/`, ~130 MB)
เก็บไว้ในเครื่องเท่านั้น (`.gitignore`) ผลที่วิเคราะห์จาก bag อยู่ใน `results/` แล้ว

## สิ่งที่ต้องมี

- ROS 2 Humble + `nav2_bringup`, `nav2_simple_commander`, TurtleBot3 packages, `TURTLEBOT3_MODEL=burger`, `ROS_DOMAIN_ID=30`
- overlay `tf2` 0.25.24 ที่ `~/tf2_fix_ws` (apt `ros-humble-tf2` 0.25.23 มี deadlock ทำให้ RViz/costmap ค้าง) ถ้าไม่มี ให้ลบบรรทัด `source $HOME/tf2_fix_ws/...` ใน `start_nav.sh`
- Python: `numpy`, `matplotlib` (`analyze.py` รันด้วย `python3 -s` เพื่อเลี่ยง numpy 2 ใน `~/.local`)
- ไม่ต้องใช้ scipy (คำนวณทุกอย่างด้วย numpy)

## ไฟล์

| ไฟล์ | ใช้ทำอะไร |
|---|---|
| `params/baseline.yaml` | Nav2 **ก่อนแก้**: ค่าจาก HW7 + แก้บั๊ก `use_sim_time` + ตรึงความเร็ว 0.10 m/s |
| `params/tuned.yaml` | Nav2 **หลังปรับจูน**: `/scan_fixed` + AMCL 0.05/0.05 + inflation 0.20/scale 10 (ใช้คู่ `maps/room_icp.yaml`) |
| `params/scanfix.yaml` | เหมือน baseline แต่อ่าน `/scan_fixed` (แก้มุม LiDAR อย่างเดียว) |
| `maps/room_icp.yaml` | แผนที่ใหม่จาก ICP (`tools/build_map.py`) ขนาดตรงกับห้องจริง แกน x = แนวเทป, `compare_maps.png` เทียบกับแผนที่เดิม |
| `tools/scan_rotate.py` | relay `/scan` → `/scan_fixed` หมุน +1.8° (`start_nav.sh` เปิดให้เองถ้า params ใช้ `scan_fixed`) |
| `tools/snapshot_config.sh` | เก็บค่าที่ใช้จริงของแต่ละชุด (params, map, `ros2 param dump`, TF, แบต) → `results/<cond>/config/` |
| `params/amclfix.yaml` | Nav2 **หลังแก้ (A)**: เหมือน baseline แต่ AMCL `update_min_d` 0.25→0.05, `update_min_a` 0.2→0.05 |
| `robot/burger_before.yaml` | param บนหุ่น **ก่อนแก้** (รัศมีล้อเดิม 0.033 m) |
| `robot/burger_after.yaml` | param บนหุ่น **หลังแก้ (B)**: รัศมีล้อ (`calibrate_odom.py --write` เขียนให้) **ผลวัดบอกว่าไม่จำเป็น** |
| `robot/fix_scan_yaw.sh` | **หลังแก้ (C)** รันบนหุ่น: ใส่มุมเอียงของ LiDAR ใน URDF (`./fix_scan_yaw.sh 1.8`, คืนค่า `./fix_scan_yaw.sh 0`) |
| `start_nav.sh` | เปิด Nav2 + RViz: `./start_nav.sh baseline` หรือ `./start_nav.sh amclfix` |
| `run_experiment.py` | รันการทดลองอัตโนมัติ 4 ระยะ × 5 รอบ สุ่มลำดับ หุ่นกลับจุดเริ่มเอง วัดตำแหน่งจริงด้วย ICP → `results/<condition>/trials.csv` |
| `calibrate_odom.py` | สั่งหุ่นวิ่งไป-กลับ 1 m (3 ความเร็ว × 2) + หมุน ±170° พร้อมบันทึก bag → วิเคราะห์ด้วย `tools/bag_calibrate.py` |
| `tools/bag_calibrate.py` | ใช้ LiDAR + กำแพงเป็นค่าจริง (ICP scan ตอนนิ่งก่อน/หลังวิ่ง) หา real/odom, มุม LiDAR, การเลี้ยวเอง ใช้กับ bag เก่าได้ด้วย |
| `analyze.py` | สรุปผล → `results/summary.md`, `summary.csv`, `figs/*.png` |

ความต่างระหว่างไฟล์ก่อนและหลังแก้ ดูได้ด้วย
`diff params/baseline.yaml params/amclfix.yaml` และ `diff robot/burger_before.yaml robot/burger_after.yaml`

## สรุปสิ่งที่แก้: ก่อนและหลังปรับจูน

| # | แก้อะไร | ที่ไหน | ก่อน (baseline) | หลัง (tuned) | เหตุผล / ผลที่คาด |
|---|---|---|---|---|---|
| 1 | ชดเชยมุม LiDAR | `tools/scan_rotate.py` → `/scan_fixed` (แก้ถาวรได้ด้วย `robot/fix_scan_yaw.sh` บนหุ่น) | AMCL/costmap ใช้ `/scan` ที่เอียง 1.8° | `scan_topic: scan_fixed`, costmap `topic: /scan_fixed` | LiDAR ติดเอียง +1.8° (วัดจาก bag) ทำให้ทิศที่ AMCL เห็นไม่ตรงกับทิศหุ่น → หุ่นเยื้องข้าง ~3 cm/m |
| 2 | แผนที่ใหม่ | `maps/room_icp.yaml` (`tools/build_map.py`) | `maps/square_room.yaml` (ช่อง 5 cm, ยาวในแผนที่ 1.55 m) | ช่อง 2.5 cm ยาว 1.675 m, แกน x = แนวเทป | แผนที่เดิมสั้นกว่าห้องจริง ~10 cm และเอียง 2–3° → AMCL บอกระยะน้อยกว่าจริง (HW7 ข้อ 3.2 และ baseline รอบแรก: AMCL 96 cm แต่จริง 108 cm) |
| 3 | inflation ของ costmap | `params/tuned.yaml` local + global | local 1.0 m / global 0.55 m, cost_scaling 3 | 0.20 m, cost_scaling 10 | จุดเริ่มห่างกำแพงซ้าย 21 cm แต่ cost ยังสูง ~180 → planner วางทางอ้อมเข้ากลางห้อง หุ่นหักขวา −28° ตอนออกตัว |
| 4 | ความถี่ที่ AMCL อัปเดต | `params/tuned.yaml` amcl | `update_min_d` 0.25 m, `update_min_a` 0.2 rad | 0.05 m, 0.05 rad | ระยะ 0.25 m แทบไม่อัปเดตเลย ตำแหน่งที่ Nav2 ใช้ระหว่างทางจึงมาจาก odom ล้วน |
| – | รัศมีล้อ | `robot/burger_after.yaml` | 0.033 | **ไม่แก้** | วัดแล้วคลาดไม่ถึง 1% |
| – | บั๊ก `use_sim_time: true` | `params/*.yaml` | แก้ตั้งแต่ baseline | – | ใช้ทั้งก่อนและหลังปรับ ไม่นับเป็นตัวแปร |

ตัวแปรควบคุมเหมือนกันทั้งก่อนและหลังปรับ ได้แก่ ระยะ 4 ระยะ × 5 รอบ ลำดับสุ่มเดียวกัน (seed 8), ความเร็ว 0.10 m/s, 1.0 rad/s,
goal tolerance, timeout, จุดเริ่ม (กลับเองด้วย ICP) และวิธีวัด (ICP เทียบ scan อ้างอิงเดียวกัน)
ค่าที่ node ใช้จริงของแต่ละชุดอยู่ใน `results/<cond>/config/` (`tools/snapshot_config.sh`) และ bag อยู่ใน `bags/`

## ผลก่อนและหลังปรับ (หุ่นจริง 14 ต.ค. 2569, ระยะละ 5 รอบ, ตำแหน่งจริงวัดด้วย ICP)

| ระยะ | baseline: จอดห่างเป้า (cm) เฉลี่ย / มัธยฐาน | tuned: เฉลี่ย / มัธยฐาน | baseline: Nav2 เชื่อผิด (cm) | tuned: Nav2 เชื่อผิด |
|---|---|---|---|---|
| 0.25 m | 2.9 / 3.5 | 4.1 / 4.4 | 2.6 | 1.4 |
| 0.50 m | 8.0 / 5.7 | 4.8 / 5.0 | 8.9 | 0.7 |
| 0.75 m | 27.3 / 17.8 | 4.3 / 4.2 (p = 0.04) | 24.7 | 1.5 |
| 1.00 m | 20.5 / 21.8 | 8.4 / 4.6 | 20.6 | 2.4 |
| ความชัน error ต่อระยะ | 28.8 cm/m (p = 0.004) | 5.0 cm/m (p = 0.26 ไม่มีนัยสำคัญ) | 28.0 cm/m | 1.4 cm/m |
| Nav2 รายงานสำเร็จ | 19/20 | 14/20 | | |

- **ก่อนปรับ:** ความคลาดเพิ่มตามระยะ (~29 cm/m) และเกือบทั้งหมดเป็นการเยื้องขวา ที่ Nav2 เชื่อผิดเท่ากับที่จอดพลาดจริง แปลว่าปัญหาอยู่ที่ localization ไม่ใช่ล้อ (odom คลาด < 2.5 cm)
- **หลังปรับ:** หุ่นจอดห่างเป้า ~4–5 cm ทุกระยะ ไม่ขึ้นกับระยะแล้ว ที่เหลือเท่ากับ goal tolerance 5 cm (หุ่นหยุดทันทีที่เข้าในรัศมี 5 cm) ส่วนที่ Nav2 เชื่อผิดเหลือ 0.7–2.4 cm
- **ผลข้างเคียง:** 5 จาก 20 รอบ หุ่นถึงตำแหน่งแล้ว (คลาด < 6 cm) แต่หันเอียง 34–44° จนหมดเวลา 60 s คือหมุนจัดทิศที่เป้าไม่สำเร็จ อีก 1 รอบ (1.00 m) ติดกลางทางที่ 74 cm แต่ Nav2 ก็รู้ว่ายังไม่ถึง
  ข้อสันนิษฐาน: AMCL อัปเดตทุก 0.05 rad ระหว่างหมุนทำให้ตำแหน่งกระโดดเข้าออกขอบ xy tolerance และคำสั่งมาถึงช้าผ่าน WiFi ทำให้หมุนเลย
  ข้อเสนอ: คืน `update_min_a` เป็น 0.2 (คง `update_min_d` 0.05) หรือขยาย `yaw_goal_tolerance`
- **ข้อจำกัด:**
  - baseline รอบ 12–20 คำสั่งไปถึงหุ่นช้า ~0.9 s (WiFi + bag recorder) หุ่นจึงเริ่มด้วยมุมเอียง 1–6° ค่าเริ่มจริงบันทึกไว้ใน `start_yaw_deg`
  - ชุด tuned ใช้แบตใหม่ (12.4 V)
  - ชุด tuned เปลี่ยนหลายอย่างพร้อมกัน ผลของแต่ละอย่างแยกจากตัวชี้วัด: Nav2 เชื่อผิด = localization (แผนที่ + LiDAR + AMCL), เยื้องข้างระหว่างทาง = inflation

กราฟ: `results/figs/compare_e_stop.png`, `errors_vs_distance_*.png`, `final_positions_*.png`; ตารางเต็ม: `results/summary.md`

## ตรวจด้วยตลับเมตร (ระยะละ 3 รอบ × 2 ชุด, `results/tape_check/`)

| | ICP | odom | AMCL | Nav2 เชื่อ |
|---|---|---|---|---|
| ก่อนจูน: คลาดจากตลับเมตรเฉลี่ย (cm) | 1.7 | 2.1 | 18.8 (32.0 ที่ 1 m) | 13.8 (28.2 ที่ 1 m) |
| หลังจูน: คลาดจากตลับเมตรเฉลี่ย (cm) | 1.3 | 1.5 | 1.7 | 1.7 |

ICP ใช้เป็นค่าจริงได้ (คลาด 1.3–1.7 cm) หุ่นจอดห่างเป้าตามตลับเมตรที่ 1.00 m: ก่อนจูน 28.0 cm, หลังจูน 4.2 cm
รันใหม่ได้ด้วย `./run_experiment.py ... --tape-wait --repeats 3` และ `./tools/tape_compare.py` สรุปฉบับเต็มอยู่ที่ `สรุป_Assignment8.md`

## ผลปรับเทียบหุ่นจริง (14 ต.ค. 2569)

ได้จาก `results/calib/calib_20261014_171621/` (bag: `results/calib_bags/calib_20261014_171621`)
ใช้ LiDAR + กำแพงห้องเป็นค่าจริง ทดสอบใน Gazebo แล้วได้ค่าถูกต้อง (real/odom 0.9995, มุม LiDAR 0.05°)

| วัดอะไร | ผล | สรุป |
|---|---|---|
| ระยะจริง / odom (14 ช่วง × 1 m) | เดินหน้า 0.996, ถอยหลัง 0.988, รวม 0.993 | รัศมีล้อแทบไม่คลาด (< 1 cm ที่ 1 m) **ไม่ต้องแก้ (B)** |
| ทิศที่หุ่นเคลื่อนจริง เทียบแกน LiDAR | −1.83° ± 0.48 (เดินหน้า −2.0°, ถอยหลัง −1.6°) | **LiDAR ติดเอียงไปทางซ้าย ~+1.8°** เดินหน้าและถอยหลังได้เครื่องหมายเดียวกัน จึงเป็นมุมติดตั้ง ไม่ใช่ล้อลื่น (bag เก่า 5 ต.ค. ได้ +2.9°) |
| หุ่นเลี้ยวเองตอนสั่งตรง | 0.05 m/s: ~0°/m, 0.10: +1.1°/m, 0.15: +2.1°/m (ถอยหลังเลี้ยวขวา) | ล้อขวาเร็วกว่าเล็กน้อย ขึ้นกับความเร็ว Nav2 แก้เองได้เพราะ IMU เห็น |
| odom (ล้อ + IMU) เห็นการเลี้ยวไหม | เห็น แต่บวกเกินประมาณ +1° ต่อรอบ (gyro bias) หมุน 172° คลาด 0.7% | ไม่ใช่ปัญหาหลัก |

ผลของ LiDAR เอียง: ทิศที่ AMCL บอกคือทิศของ LiDAR ไม่ใช่ทิศของหุ่น หุ่นจึงเคลื่อนเยื้องจากทิศที่ Nav2 คิดไป ~1.8°
คิดเป็นเยื้องข้างประมาณ 3 cm ต่อ 1 m (เพิ่มตามระยะ) Nav2 เห็นหุ่นไหลออกข้างเลยต้องหักกลับไปมา **จึงดูเหมือนวิ่งเฉ**
แผนที่ `square_room` สร้างด้วย TF ที่ยังไม่แก้ กำแพงในแผนที่จึงเอียง ~2–3° ไปด้วย (เห็นได้ใน `.pgm`)
`run_experiment.py` จึงคำนวณแนวเทปใน map จากทิศที่ AMCL เห็นตอนเริ่ม หักด้วยมุม LiDAR (`SCAN_YAW_DEG`)
เป้าหมายจึงอยู่บนแนวเทปจริง และค่าตลับเมตรแปลงเข้า map ได้ถูก

## ตัวแปร

**ตัวแปรต้น:** ระยะทาง 0.25 / 0.50 / 0.75 / 1.00 m เดินเส้นตรง เริ่มจาก (0.10, −0.40) ไปทาง +x
(เป้าที่ไกลสุดคือ x = 1.10 ห่างผิวกำแพงประมาณ 31 cm ใน HW7 เป้า 1.20 m ชิดกำแพงจนเกิด recovery)

**ตัวแปรควบคุม:**

| ตัวแปร | ค่า |
|---|---|
| ความเร็วสูงสุด / ความเร็วหมุนสูงสุด | 0.10 m/s / 1.0 rad/s (สคริปต์ตั้งค่าซ้ำทุกรอบ) |
| จุดเริ่มและทิศ | (0.10, −0.40) หันตามแนวเทป วางหุ่นชิดเทป L เดิมจาก HW7 เป้าอยู่บนแนวเทป (ดูหัวข้อปรับเทียบ) |
| Goal tolerance | xy 0.05 m, yaw 0.10 rad |
| Progress checker | 0.05 m / 20 s |
| Timeout | 60 s |
| Initial pose | สคริปต์ตั้งค่าพร้อม covariance (σ 3 cm, 3°) แล้วบังคับ AMCL อัปเดต 6 ครั้งให้ลู่เข้ากำแพงก่อนออกตัว |
| แผนที่และห้อง | `square_room` ไม่มีคนหรือของอยู่ในระยะที่ LiDAR มองเห็น |
| แบตเตอรี่ | ≥ 11.5 V (สคริปต์เตือนเมื่อต่ำกว่า) |
| LiDAR | พัก 5 นาทีทุก 10 รอบ (สคริปต์เตือน) |
| ลำดับรอบ | สุ่มด้วย seed 8 ได้ลำดับเดียวกันทุก condition (`results/<cond>/schedule.csv`) |

**ตัวแปรตาม (บันทึกทุกรอบ):** ผลลัพธ์จาก Nav2, เวลา, recovery, จำนวนครั้งที่ AMCL อัปเดต, แรงดันแบต,
ตำแหน่งที่ Nav2 เชื่อตอนหยุด (TF `map→base_footprint`), AMCL หลังบังคับอัปเดต, odom, **ตำแหน่งจริงจากตลับเมตร**

| ค่า error | ความหมาย |
|---|---|
| `e_stop` (`_long`, `_lat`) | หุ่นจอดห่างเป้าจริงเท่าไร (ค่าหลัก) แยกตามแนวยาว (เกิน/ขาด) และแนวข้าง |
| `e_tf` | ตำแหน่งที่ Nav2 ใช้ตัดสินว่าถึงเป้า คลาดจากความจริงเท่าไร |
| `e_amcl`, `e_odom` | ใช้แยกว่าความคลาดมาจาก AMCL หรือจาก odom |
| `odom_scale` | ระยะที่ odom นับได้ ÷ ระยะจริง (> 1 = odom นับเกิน) |

## วิธีวัดตำแหน่งจริง (อัตโนมัติด้วย LiDAR ICP)

- **scan อ้างอิง** = scan ตอนคนวางหุ่นบนเทปจุดเริ่ม (ช่วงนิ่งแรกของ bag ปรับเทียบ) เก็บไว้ที่ `results/ref_scan.npz`
  ใช้กำหนด**กรอบเทป**: (0,0) = จุดเริ่ม, x = ตามเทป, y = ซ้าย
- ทุกครั้งที่หุ่นหยุด สคริปต์เอา median ของ 5 scan มา ICP เทียบ scan อ้างอิง แล้วแปลงเป็นตำแหน่งหุ่นด้วยมุม LiDAR จริง 1.8°
  วัดซ้ำได้ ±0.03 cm / ±0.1° ใน Gazebo ได้ค่าตรงกับค่าจริง และตรงกับ odom ภายใน 1%
- ใช้ `/scan` ดิบเสมอ ไม่ได้ใช้ TF หรือ relay ที่ Nav2 ใช้ จึงเทียบค่าก่อนและหลังปรับได้ตรง ๆ
- **กลับจุดเริ่มเอง:** หุ่นขับไปจุดพัก (0.30, 0) แล้วถอยตรงเข้า (0,0) โดยใช้ ICP นำทาง จนคลาดไม่เกิน 1.2 cm และ 0.6°
  ระหว่างทางเช็กช่องทางวิ่ง และหยุดเองถ้าเจอของใกล้
- ตลับเมตรยังใช้ได้ด้วย `--manual` (ให้คนวางหุ่นเองทุกรอบ)

## ขั้นตอน

ทุก terminal บน PC ต้องรัน `source /opt/ros/humble/setup.bash && source ~/tf2_fix_ws/install/setup.bash && export ROS_DOMAIN_ID=30 TURTLEBOT3_MODEL=burger` ก่อน
และบนหุ่นต้องรอให้ `timedatectl` ขึ้น `System clock synchronized: yes` ก่อนเปิด bringup

### ข้อ 1: ก่อนแก้ (baseline)
```bash
# บนหุ่น: ใช้ค่าเดิม
ros2 launch turtlebot3_bringup robot.launch.py
# PC terminal 1
./start_nav.sh baseline
# PC terminal 2 (เก็บค่าที่ใช้จริง + bag)
./tools/snapshot_config.sh baseline params/baseline.yaml maps/square_room.yaml
ros2 bag record -o bags/baseline /odom /scan /amcl_pose /cmd_vel /cmd_vel_nav /plan /local_plan /tf /tf_static /battery_state /initialpose /imu
# PC terminal 3
./run_experiment.py --condition baseline
```

จากนั้นรัน `./analyze.py` แล้วดูผลใน `results/summary.md` และ `results/figs/` เพื่อหาเหตุ:

| ถ้าผลออกมาแบบนี้ | แปลว่า | ใช้วิธีแก้ |
|---|---|---|
| ระยะ 0.25–0.50 m `e_stop`/`e_tf` สูง และ AMCL อัปเดต 0–2 ครั้ง | AMCL แทบไม่อัปเดตในระยะสั้น ตำแหน่งที่ Nav2 ใช้ไม่ตรงกับจริง | **A: amclfix** |
| `e_stop_lat` (เยื้องข้าง) หรือมุมจริงเบี้ยวเพิ่มตามระยะ ~3 cm/m | LiDAR ติดเอียง ~1.8° (วัดแล้ว หัวข้อปรับเทียบ) | **C: scanfix** |
| `e_odom_long` เพิ่มเป็นเส้นตรงตามระยะ และ `odom_scale` ≠ 1 ทุกระยะ | ค่ารัศมีล้อใน param ไม่ตรงกับล้อจริง (วัดแล้วคลาด < 1%) | B: odomcal |

### ข้อ 2 (หลังปรับจูน): tuned = แก้ทุกสาเหตุที่วัดได้ (แก้บน PC อย่างเดียว ไม่ต้องแก้หุ่น)
```bash
./start_nav.sh tuned            # params/tuned.yaml + maps/room_icp.yaml + scan_rotate.py (/scan_fixed)
./tools/snapshot_config.sh tuned params/tuned.yaml maps/room_icp.yaml
ros2 bag record -o bags/tuned ...     # topic เดียวกับ baseline + /scan_fixed
./run_experiment.py --condition tuned --scan-fixed --start 0 0 --init-yaw 0
```
`--start 0 0 --init-yaw 0` ใช้เพราะ `room_icp` สร้างในกรอบเทป จุดเริ่มจึงอยู่ที่ (0,0) และหุ่นบนเทปหัน 0°

### ข้อ 2-A: แก้ AMCL อย่างเดียว (แก้บน PC อย่างเดียว ไม่ต้องแก้หุ่น)
```bash
./start_nav.sh amclfix                                  # PC terminal 1 (ปิดตัวเดิมก่อน)
python3 run_experiment.py --condition amclfix                 # PC terminal 2
```

### ข้อ 2-C: แก้มุม LiDAR (แก้บนหุ่น: URDF)
TF `base_link → base_scan` มาจาก `robot_state_publisher` **บนหุ่น** ซึ่งอ่าน
`~/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf` (joint `scan_joint`)
```bash
scp robot/fix_scan_yaw.sh ubuntu@<IP หุ่น>:~/
ssh ubuntu@<IP หุ่น>
./fix_scan_yaw.sh 1.8                  # สำรองเป็น .urdf.orig, แก้ rpy="0 0 0.03142", build turtlebot3_description
# ปิด bringup เดิม (Ctrl+C) แล้วเปิดใหม่
ros2 launch turtlebot3_bringup robot.launch.py
```
เช็กบน PC: `ros2 run tf2_ros tf2_echo base_link base_scan` → yaw ต้องเป็น ~1.8°
ตรวจผลแก้ด้วยการวิ่งชุดเดิม: `python3 calibrate_odom.py` → แถว "ทิศที่เคลื่อนจริงเทียบแกน LiDAR" ต้องเหลือ ~0°
ทดลอง: `./start_nav.sh baseline` + `python3 run_experiment.py --condition scanfix --scan-fixed`
(`--scan-fixed` บอกสคริปต์ว่า TF ถูกแล้ว ไม่ต้องชดเชยมุมเอง ตอนเริ่มจะตั้ง initial yaw −1.8° เพราะแผนที่สร้างตอน TF ยังเอียง)
คืนค่าเดิม: `./fix_scan_yaw.sh 0` แล้วเปิด bringup ใหม่

### ข้อ 2-B (ไม่แนะนำ: วัดแล้วคลาด < 1%): แก้รัศมีล้อ
`python3 calibrate_odom.py --analyze-only results/calib_bags/calib_20261014_171621 --write` จะเขียน `robot/burger_after.yaml`
แล้วส่งขึ้นหุ่นและรัน `ros2 launch turtlebot3_bringup robot.launch.py tb3_param_dir:=/home/ubuntu/burger_after.yaml`
(ก่อนใช้ ให้ `diff` เทียบกับ `burger.yaml` ของหุ่นก่อน ต้องต่างกันแค่บรรทัด radius)

(ถ้าต้องการ) รวมหลายวิธีพร้อมกัน เช่น `./start_nav.sh amclfix` + LiDAR แก้แล้ว + `--condition both --scan-fixed`

### วิเคราะห์ก่อนและหลังแก้
```bash
./analyze.py          # baseline = ก่อนแก้ เทียบกับทุก condition อื่น
```
`results/summary.md` มีตารางค่าเฉลี่ย ± SD ต่อระยะ, ความชันของ error ต่อระยะ (cm/m, R²)
และตารางเทียบ `e_stop` ก่อนและหลังแก้ พร้อม % ที่เปลี่ยนและ Welch t-test

`figs/` มีกราฟต่อไปนี้
- `errors_vs_distance_<cond>.png`
- `error_components_<cond>.png`
- `final_positions_<cond>.png`
- `amcl_updates.png`
- `compare_e_stop.png`, `compare_e_odom.png`

## ที่แก้จาก HW7
- `global_costmap` และ `behavior_server` เคยตั้ง `use_sim_time: true` บนหุ่นจริง ตอนนี้แก้เป็น `False` แล้ว
- ไม่เปลี่ยนความเร็วและเส้นทาง และไม่ใช้ `max_vel_theta = 10·v` แล้ว
- เลิกใช้ระยะ 1.20 m เพราะเป้าชิดกำแพง
- วัดตำแหน่งจริงทุกรอบ และบันทึกตำแหน่งที่ Nav2 เชื่อ (TF) ตอนหยุด แยกจากค่า AMCL หลังบังคับอัปเดต
- สุ่มลำดับรอบ บันทึกแรงดันแบต และพัก LiDAR เป็นช่วง
- เป้าหมายอยู่บนแนวเทปจริง (HW7 ใช้แกน x ของ map ซึ่งเอียงจากเทป ~2° ตามมุม LiDAR) ค่าตลับเมตรจึงเทียบกับเป้าได้ตรง

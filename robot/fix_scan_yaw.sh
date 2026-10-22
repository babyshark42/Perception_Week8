#!/bin/bash
# รันบนหุ่น (Raspberry Pi): ใส่มุมเอียงของ LiDAR ลงใน URDF ให้ TF base_link -> base_scan ถูก
#   ./fix_scan_yaw.sh 1.8      ใส่ yaw +1.8° (ค่าจาก calibrate_odom.py, + = LiDAR หันไปทางซ้ายของหุ่น)
#   ./fix_scan_yaw.sh 0        คืนค่าเดิม (ก่อนแก้)
# จากนั้นปิดแล้วเปิด bringup ใหม่: ros2 launch turtlebot3_bringup robot.launch.py
set -e
DEG=${1:?ใส่มุมเป็นองศา เช่น ./fix_scan_yaw.sh 1.8}
RAD=$(python3 -c "import math; print(round(math.radians(float('$DEG')), 5))")
WS=${TB3_WS:-$HOME/turtlebot3_ws}
URDF=$(find "$WS/src" -path '*turtlebot3_description/urdf/turtlebot3_burger.urdf' | head -1)
[ -n "$URDF" ] || { echo "ไม่พบ turtlebot3_burger.urdf ใน $WS/src (ตั้ง TB3_WS=... ถ้า workspace อยู่ที่อื่น)"; exit 1; }
[ -f "$URDF.orig" ] || cp "$URDF" "$URDF.orig"           # สำรองไฟล์เดิมครั้งแรกครั้งเดียว
# แก้เฉพาะ <origin> ใน joint scan_joint
sed -i "/scan_joint/,/<\/joint>/ s|rpy=\"[^\"]*\"|rpy=\"0 0 $RAD\"|" "$URDF"
grep -A3 'scan_joint' "$URDF" | grep origin
source /opt/ros/humble/setup.bash
cd "$WS" && colcon build --symlink-install --packages-select turtlebot3_description
echo "เสร็จ: scan_joint yaw = $DEG° ($RAD rad) -> เปิด bringup ใหม่ แล้วเช็ก:"
echo "  ros2 run tf2_ros tf2_echo base_link base_scan"

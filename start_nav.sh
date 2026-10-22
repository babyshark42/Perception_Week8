#!/bin/bash
# Nav2 (no RViz) + RViz แยก (ปิด LaserScan display กัน deadlock)
# ใช้:  ./start_nav.sh            -> params/baseline.yaml  (ข้อ 1)
#       ./start_nav.sh amclfix    -> params/amclfix.yaml   (ข้อ 2-A)
#       ./start_nav.sh scanfix    -> params/scanfix.yaml   (ข้อ 2-C: หมุน /scan 1.8° บน PC -> /scan_fixed)
#       ./start_nav.sh tuned      -> params/tuned.yaml + maps/room_icp.yaml (หลังปรับจูน: รวมทุกการแก้)
#       MAP=/path/map.yaml ./start_nav.sh <cond>   เลือกแผนที่เอง
source /opt/ros/humble/setup.bash
source $HOME/tf2_fix_ws/install/setup.bash  # tf2 0.25.24: แก้ deadlock ใน 0.25.23 (apt)
export ROS_DOMAIN_ID=30 TURTLEBOT3_MODEL=burger
D=$(dirname "$(readlink -f "$0")")
COND=${1:-baseline}
PARAMS="$D/params/$COND.yaml"
if [ -z "$MAP" ]; then
  if [ "$COND" = tuned ]; then MAP=$D/maps/room_icp.yaml; else MAP=$D/maps/square_room.yaml; fi
fi
[ -f "$PARAMS" ] || { echo "ไม่พบ $PARAMS"; exit 1; }
echo "params: $PARAMS  map: $MAP"
if grep -q scan_fixed "$PARAMS"; then
  python3 "$D/tools/scan_rotate.py" --deg ${SCAN_YAW_DEG:-1.8} > /tmp/week8_scan_rotate.log 2>&1 &
  trap 'kill %1 2>/dev/null' EXIT
fi
rviz2 -d "$D/tb3_nav_nolaser.rviz" > /tmp/week8_rviz.log 2>&1 &
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False \
  map:="$MAP" params_file:="$PARAMS"

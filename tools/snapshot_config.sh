#!/bin/bash
# เก็บค่าที่ใช้จริงของแต่ละชุดการทดลอง -> results/<cond>/config/
#   ./tools/snapshot_config.sh baseline params/baseline.yaml maps/square_room.yaml
# เก็บ: ไฟล์ params + map ที่ใช้, ค่าที่ node กำลังใช้จริง (ros2 param dump), TF LiDAR, แรงดันแบต
set -e
COND=$1; PARAMS=$2; MAP=$3
D=$(dirname "$(readlink -f "$0")")/..
OUT=$D/results/$COND/config
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
cp "$PARAMS" "$OUT/"
cp "$MAP" "$OUT/"
cp "$(dirname "$MAP")/$(grep '^image:' "$MAP" | awk '{print $2}')" "$OUT/"
for n in /amcl /controller_server /planner_server /global_costmap/global_costmap /local_costmap/local_costmap; do
  f=$(echo "$n" | tr '/' '_' | sed 's/^_//')
  timeout 15 ros2 param dump "$n" > "$OUT/live_$f.yaml" 2>/dev/null || echo "dump $n ไม่ได้"
done
timeout 8 ros2 run tf2_ros tf2_echo base_link base_scan 2>/dev/null | head -8 > "$OUT/tf_base_scan.txt" || true
timeout 5 ros2 topic echo --once /battery_state 2>/dev/null | grep -m1 voltage > "$OUT/battery.txt" || true
pgrep -af scan_rotate.py > "$OUT/scan_rotate.txt" || echo "scan_rotate ไม่ได้รัน" > "$OUT/scan_rotate.txt"
date '+%F %T' > "$OUT/snapshot_time.txt"
echo "เก็บค่า $COND -> $OUT"
grep -h -E "update_min_d|update_min_a|scan_topic|inflation_radius|cost_scaling_factor" "$OUT"/live_*.yaml | sort | uniq -c

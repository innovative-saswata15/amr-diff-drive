#!/usr/bin/env bash
# Pre-flight check for amr_navigation: run AFTER the robot bringup is up,
# BEFORE starting SLAM / Nav2.
#   ros2 run amr_navigation check_setup.sh
#   ros2 run amr_navigation check_setup.sh /my_odom_topic /my_scan_topic

ODOM_TOPIC="${1:-/diff_drive_controller/odom}"
SCAN_TOPIC="${2:-/scan}"
PASS=0; FAIL=0

ok()   { echo -e "  [ OK ] $1"; PASS=$((PASS+1)); }
bad()  { echo -e "  [FAIL] $1"; FAIL=$((FAIL+1)); }
info() { echo -e "  [info] $1"; }

echo "== amr_navigation pre-flight check =="

echo "-- Topics"
TOPICS=$(ros2 topic list 2>/dev/null)
for t in "$SCAN_TOPIC" "$ODOM_TOPIC" /tf /tf_static /encoder_telemetry; do
  if echo "$TOPICS" | grep -qx "$t"; then ok "$t exists"; else bad "$t missing"; fi
done

echo "-- Data actually flowing (3 s each)"
for t in "$SCAN_TOPIC" "$ODOM_TOPIC"; do
  if timeout 3 ros2 topic echo --once "$t" >/dev/null 2>&1; then
    ok "$t is publishing"
  else
    bad "$t published nothing in 3 s"
  fi
done

echo "-- diff_drive_controller command topic"
if echo "$TOPICS" | grep -qx /diff_drive_controller/cmd_vel_unstamped; then
  info "controller listens on /diff_drive_controller/cmd_vel_unstamped (Twist)"
  info "=> launch with: relay_stamped:=false relay_output_topic:=/diff_drive_controller/cmd_vel_unstamped"
elif echo "$TOPICS" | grep -qx /diff_drive_controller/cmd_vel; then
  info "controller listens on /diff_drive_controller/cmd_vel (TwistStamped) => default relay settings are correct"
else
  bad "no /diff_drive_controller/cmd_vel* topic: is the controller active? (ros2 control list_controllers)"
fi

echo "-- TF: odom -> base_link"
if timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation"; then
  ok "odom -> base_link available"
else
  bad "odom -> base_link NOT available (check enable_odom_tf / frame names)"
fi

echo "-- TF: base_link -> LiDAR frame"
SCAN_FRAME=$(timeout 3 ros2 topic echo --once "$SCAN_TOPIC" 2>/dev/null | grep frame_id | head -1 | awk '{print $2}' | tr -d "'\"")
if [ -n "$SCAN_FRAME" ]; then
  info "scan frame_id = $SCAN_FRAME"
  if timeout 4 ros2 run tf2_ros tf2_echo base_link "$SCAN_FRAME" 2>&1 | grep -q "Translation"; then
    ok "base_link -> $SCAN_FRAME available"
  else
    bad "base_link -> $SCAN_FRAME NOT available (static TF / URDF missing)"
  fi
else
  bad "could not read frame_id from $SCAN_TOPIC"
fi

echo "-- Who publishes /tf (must be exactly ONE odom->base_link source)"
ros2 topic info /tf -v 2>/dev/null | grep "Node name" | sed 's/^/  [info] /'

echo
echo "== Result: $PASS passed, $FAIL failed =="
[ "$FAIL" -eq 0 ] && echo "Ready for SLAM / Nav2." || echo "Fix the FAIL items first (see README section 9)."

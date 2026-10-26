#!/usr/bin/env bash
# End-to-end check inside the ROS 2 Docker image (used by CI):
# simulator -> SLCAN -> ROS 2 driver -> /joint_states, /diagnostics, services.
set -euo pipefail
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

/ws/repo/firmware/build/actuator_sim --port 29536 --state-dir /tmp --quiet &
SIM=$!
sleep 1

# Bring-up result the driver relies on: node 2's motor polarity is reversed.
python3 -m bringup params --node 2 --set motor_polarity=-1 --save

ros2 launch actuator_driver driver.launch.py &
DRIVER=$!
trap 'kill $DRIVER $SIM 2>/dev/null || true' EXIT

for i in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q /actuator_driver/enable; then break; fi
  sleep 1
done

echo "--- /joint_states"
timeout 10 ros2 topic echo --once /joint_states
echo "--- services"
ros2 service call /actuator_driver/clear_faults std_srvs/srv/Trigger
ros2 service call /actuator_driver/enable std_srvs/srv/Trigger
echo "--- motion and diagnostics check"
python3 /ws/src/actuator_driver/test/ros_check.py
ros2 service call /actuator_driver/disable std_srvs/srv/Trigger
echo "ROS 2 CHECK PASSED"

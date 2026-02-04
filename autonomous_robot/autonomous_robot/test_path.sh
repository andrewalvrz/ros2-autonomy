#!/bin/bash
set -e

cmd() {
  local MSG="$1"
  local DUR="$2"

  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "$MSG" &
  PID=$!
  sleep "$DUR"
  kill "$PID" 2>/dev/null || true
}

# 34.5" forward @ 0.20 m/s -> 4.38 s
cmd "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 4.38

# left 180 deg @ 0.50 rad/s -> 6.28 s
cmd "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" 6.28

# 11" forward -> 1.40 s
cmd "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 1.40

# left 180 deg again
cmd "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" 6.28

# 28" forward -> 3.56 s
cmd "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 3.56

# STOP (send once)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

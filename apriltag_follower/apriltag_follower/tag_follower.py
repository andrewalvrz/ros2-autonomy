#!/usr/bin/env python3
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pupil_apriltags import Detector


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class AprilTagFollower(Node):
    """
    ROS2 node: webcam -> AprilTag -> /cmd_vel
      - If tag seen: turn to center + move forward
      - If tag lost: spin to search
      - Stop "before" tag using tag pixel area threshold (proxy distance)
    """

    def __init__(self):
        super().__init__("apriltag_follower")

        # ---------------- Parameters ----------------
        # Use device path for reliability on Linux/Jetson, e.g. "/dev/video4"
        self.declare_parameter("camera_device", "/dev/video4")  # <-- default set to /dev/video4
        self.declare_parameter("camera_index", 0)               # fallback if camera_device is ""
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)

        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("target_id", -1)  # -1 = follow any tag, else follow that ID only

        # Motion tuning
        self.declare_parameter("max_vx", 0.20)   # m/s
        self.declare_parameter("max_wz", 0.70)   # rad/s
        self.declare_parameter("kp_ang", 0.80)   # steering gain (lower = less spin/oscillation)

        # Stop/slow based on tag area (pixel^2) - tune via logs
        self.declare_parameter("slow_area", 9000.0)
        self.declare_parameter("stop_area", 14000.0)

        # Search behavior
        self.declare_parameter("search_wz", 0.55)        # rad/s spin when lost
        self.declare_parameter("lost_timeout_s", 0.35)   # seconds w/out tag => search
        self.declare_parameter("publish_hz", 20.0)

        # Extra stability
        self.declare_parameter("center_deadband", 0.06)  # normalized error deadband around center
        self.declare_parameter("hard_turn_err", 0.35)    # if |err| > this, slow forward a lot

        # ---------------- ROS pubs ----------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---------------- AprilTag Detector ----------------
        fam = self.get_parameter("tag_family").value
        self.detector = Detector(
            families=fam,
            nthreads=2,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        # ---------------- Camera ----------------
        dev = str(self.get_parameter("camera_device").value).strip()
        cam_idx = int(self.get_parameter("camera_index").value)
        w_req = int(self.get_parameter("width").value)
        h_req = int(self.get_parameter("height").value)
        fps_req = int(self.get_parameter("fps").value)

        if dev:
            self.get_logger().info(f"Opening camera device: {dev}")
            self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        else:
            self.get_logger().info(f"Opening camera index: {cam_idx}")
            self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera (device='{dev}' index={cam_idx})")

        # Try to request settings (not always respected)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w_req)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h_req)
        self.cap.set(cv2.CAP_PROP_FPS, fps_req)

        # State
        self.last_seen_time = 0.0

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.loop)

        self.get_logger().info(
            f"AprilTagFollower started. family={fam}, device={dev or cam_idx}. Publishing /cmd_vel"
        )

    def publish_stop(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def publish_cmd(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = 0.0
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def choose_target(self, detections):
        """Follow target_id if set, else follow largest-area tag."""
        target_id = int(self.get_parameter("target_id").value)
        if not detections:
            return None

        if target_id >= 0:
            for d in detections:
                if d.tag_id == target_id:
                    return d
            return None

        # Follow the biggest (closest-looking) tag
        def area_of(d):
            pts = np.array(d.corners, dtype=np.float32)
            return float(cv2.contourArea(pts))

        return max(detections, key=area_of)

    def loop(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Camera frame grab failed; stopping.")
            self.publish_stop()
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets = self.detector.detect(gray, estimate_tag_pose=False)
        tgt = self.choose_target(dets)

        now = time.time()

        max_vx = float(self.get_parameter("max_vx").value)
        max_wz = float(self.get_parameter("max_wz").value)
        kp_ang = float(self.get_parameter("kp_ang").value)

        slow_area = float(self.get_parameter("slow_area").value)
        stop_area = float(self.get_parameter("stop_area").value)

        search_wz = float(self.get_parameter("search_wz").value)
        lost_timeout = float(self.get_parameter("lost_timeout_s").value)

        center_deadband = float(self.get_parameter("center_deadband").value)
        hard_turn_err = float(self.get_parameter("hard_turn_err").value)

        h, w = gray.shape[:2]
        cx_img = w * 0.5

        if tgt is not None:
            self.last_seen_time = now

            cx_tag = float(tgt.center[0])
            err = (cx_tag - cx_img) / cx_img  # normalized: left -, right +

            # Deadband to prevent jitter around center
            if abs(err) < center_deadband:
                err = 0.0

            # Turn toward the tag center
            wz = clamp(-kp_ang * err, -max_wz, max_wz)

            # Area proxy for distance
            pts = np.array(tgt.corners, dtype=np.float32)
            area = float(cv2.contourArea(pts))

            # Forward speed logic
            if area >= stop_area:
                vx = 0.0
            elif area >= slow_area:
                # ramp down as it approaches stop_area
                alpha = (stop_area - area) / max(1.0, (stop_area - slow_area))
                vx = max_vx * clamp(alpha, 0.0, 1.0) * 0.6
            else:
                vx = max_vx

            # If badly misaligned, slow forward to avoid spiraling
            if abs(err) > hard_turn_err:
                vx *= 0.20

            self.publish_cmd(vx, wz)

            self.get_logger().info(
                f"tag={tgt.tag_id} err={err:+.2f} area={area:.0f} -> vx={vx:.2f} wz={wz:.2f}",
                throttle_duration_sec=0.2
            )
            return

        # No target visible: search after timeout
        if (now - self.last_seen_time) > lost_timeout:
            self.publish_cmd(0.0, search_wz)
        else:
            self.publish_stop()

    def destroy_node(self):
        try:
            self.publish_stop()
        except Exception:
            pass
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import time

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class CmdVelSerialBridge(Node):
    def __init__(self):
        super().__init__('cmdvel_serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 20.0)

        self.declare_parameter('max_vx', 0.50)
        self.declare_parameter('max_vy', 0.50)
        self.declare_parameter('max_wz', 1.50)

        self.declare_parameter('cmd_timeout_s', 0.5)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)

        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = time.time()
        self.stale_warned = False

        self.lock = threading.Lock()

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        self.ser = None
        self.last_open_attempt = 0.0
        self.open_serial()

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Bridge running: /cmd_vel -> {self.port} @ {self.baud}, {self.rate_hz} Hz"
        )
        self.get_logger().info("Serial protocol: V,vx,vy,wz\\n")

    def open_serial(self):
        now = time.time()
        if now - self.last_open_attempt < 1.0:
            return
        self.last_open_attempt = now

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)

            # Prevent ESP32 auto-reset / bootloader weirdness on open
            try:
                self.ser.setDTR(False)
                self.ser.setRTS(False)
            except Exception:
                pass

            time.sleep(0.2)
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            self.get_logger().info("Serial opened OK")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            self.ser = None

    def on_cmd_vel(self, msg: Twist):
        vx = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        vy = clamp(msg.linear.y, -self.max_vy, self.max_vy)
        wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)

        with self.lock:
            self.vx = vx
            self.vy = vy
            self.wz = wz
            self.last_cmd_time = time.time()

    def on_timer(self):
        if self.ser is None:
            self.open_serial()
            return

        with self.lock:
            vx = self.vx
            vy = self.vy
            wz = self.wz
            age = time.time() - self.last_cmd_time

        if age > self.cmd_timeout_s:
            vx, vy, wz = 0.0, 0.0, 0.0
            if not self.stale_warned:
                self.get_logger().warn("cmd_vel stale -> sending STOP")
                self.stale_warned = True
        else:
            self.stale_warned = False

        line = f"V,{vx:.3f},{vy:.3f},{wz:.3f}\n"
        try:
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

def main():
    rclpy.init()
    node = CmdVelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

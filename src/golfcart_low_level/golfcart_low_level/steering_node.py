#!/usr/bin/env python3
"""
steering_node — KEYA KY170DD01005-08G position control over CAN (extended ID)

Listens to /steering_cmd (Float64, radians) published by ros_can_bridge
(NOT /cmd_vel — the bridge owns the safety logic) and keeps the motor's
1000 ms watchdog fed by re-sending the position frame at a fixed rate.

KEYA protocol (V2.8 manual, all frames extended 29-bit):
  command  ID 0x06000000 + motor_id
     enable    23 0D 20 01 00 00 00 00
     disable   23 0C 20 01 00 00 00 00
     position  23 02 20 01 LL_H LL_L HH_H HH_L   (int32, low word first,
                                                  high byte first in a word;
                                                  10000 counts / motor rev)
  response ID 0x05800000 + motor_id
  heartbeat ID 0x07000000 + motor_id  (big-endian words):
     [0..1] cumulative angle, 1 deg/LSB of the MOTOR shaft, wraps at 65535
     [2..3] motor speed (signed)   [4..5] motor current (signed)
     [6..7] error code (0x4001 = CAN disconnected / disabled)

Publishes:
  /steering_angle (Float64, radians)  — estimated road-wheel steering angle
  /steering_fault (Bool)              — heartbeat error code != 0 or lost

Before running:  sudo ip link set can0 up type can bitrate 250000
"""

import math
import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64

COUNTS_PER_MOTOR_REV = 10000.0     # position-mode resolution
HB_DEG_PER_LSB = 1.0               # heartbeat angle resolution (deg of motor shaft)


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')

        # ---------- parameters ----------
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('input_mode', 'steering_cmd')  # steering_cmd or cmd_vel
        self.declare_parameter('steering_cmd_topic', '/steering_cmd')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('require_lowlevel_safety', True)
        self.declare_parameter('tx_rate_hz', 20.0)          # < watchdog 1000 ms
        self.declare_parameter('cmd_timeout_s', 0.5)        # /steering_cmd เงียบ -> safe state
        self.declare_parameter('max_steering_deg', 50.0)
        # counts ต่อ 1 องศาของ "มุมเลี้ยวล้อ" (จากการ calibrate เดิมของคุณ)
        # NOTE: ค่าซ้าย/ขวาไม่เท่ากันเป็นการชดเชม zero-offset — ควร calibrate
        #       zero ที่มอเตอร์ (param 0015/0016) แล้วทำให้สองค่านี้เท่ากัน
        self.declare_parameter('counts_per_deg_pos', 180.0)
        self.declare_parameter('counts_per_deg_neg', 320.0)
        self.declare_parameter('enable_period_s', 1.0)      # ส่ง enable ซ้ำกัน motor หลุด
        self.declare_parameter('safe_steering_deg', 0.0)    # มุมที่สั่งเมื่อ timeout/fault/e-stop
        self.declare_parameter('disable_on_fault', False)   # True = ส่ง disable เมื่อ fault/e-stop
        self.declare_parameter('enable_on_startup', False)  # True = enable ก่อนเห็นสถานะ safety

        ch = self.get_parameter('can_channel').value
        mid = int(self.get_parameter('motor_id').value)
        self.input_mode = str(self.get_parameter('input_mode').value)
        self.steering_cmd_topic = str(self.get_parameter('steering_cmd_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.require_lowlevel_safety = bool(self.get_parameter('require_lowlevel_safety').value)
        self.tx_rate   = float(self.get_parameter('tx_rate_hz').value)
        self.cmd_to    = float(self.get_parameter('cmd_timeout_s').value)
        self.max_deg   = float(self.get_parameter('max_steering_deg').value)
        self.scale_pos = float(self.get_parameter('counts_per_deg_pos').value)
        self.scale_neg = float(self.get_parameter('counts_per_deg_neg').value)
        self.en_period = float(self.get_parameter('enable_period_s').value)
        self.safe_deg  = float(self.get_parameter('safe_steering_deg').value)
        self.disable_on_fault = bool(self.get_parameter('disable_on_fault').value)
        self.enable_on_startup = bool(self.get_parameter('enable_on_startup').value)

        self.id_cmd = 0x06000000 + mid
        self.id_rsp = 0x05800000 + mid
        self.id_hb  = 0x07000000 + mid

        # ---------- CAN ----------
        self.bus = can.Bus(channel=ch, interface='socketcan')

        # ---------- state ----------
        self.target_deg = 0.0            # มุมเลี้ยวเป้าหมาย (deg, +ขวา/-ซ้าย ตาม convention เดิม)
        self.last_cmd_time = None
        self.last_enable_time = 0.0
        self.hb_prev_raw = None          # heartbeat angle word ก่อนหน้า (uint16)
        self.motor_counts = 0.0          # accumulated counts ตั้งแต่ boot (0 = ตำแหน่งตอนเปิด)
        self.last_hb_time = None
        self.error_code = 0
        self.lowlevel_fault = self.require_lowlevel_safety
        self.estop_active = self.require_lowlevel_safety
        self.motor_enabled = False
        self._fault_logged = False

        # ---------- ROS I/O ----------
        if self.input_mode == 'cmd_vel':
            self.create_subscription(Twist, self.cmd_vel_topic, self._cb_cmd_vel, 10)
        elif self.input_mode == 'steering_cmd':
            self.create_subscription(Float64, self.steering_cmd_topic, self._cb_cmd, 10)
        else:
            raise ValueError("input_mode must be 'steering_cmd' or 'cmd_vel'")

        if self.require_lowlevel_safety:
            self.create_subscription(Bool, '/lowlevel_fault', self._cb_lowlevel_fault, 10)
            self.create_subscription(Bool, '/e_stop_status', self._cb_estop, 10)
        self.pub_angle = self.create_publisher(Float64, '/steering_angle', 10)
        self.pub_fault = self.create_publisher(Bool, '/steering_fault', 10)

        self.create_timer(1.0 / self.tx_rate, self._tx_loop)

        self._rx_stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        if self.enable_on_startup:
            self._send_enable()
        else:
            self.get_logger().info(
                'KEYA enable is gated until command is fresh and safety is clear')
        self.get_logger().info(
            f'steering_node up: cmd=0x{self.id_cmd:08X} hb=0x{self.id_hb:08X} '
            f'@ {self.tx_rate:.0f} Hz, input_mode={self.input_mode}, '
            f'require_lowlevel_safety={self.require_lowlevel_safety}')

    # ================= ROS =================
    def _cb_cmd(self, msg: Float64):
        deg = math.degrees(float(msg.data))
        self.target_deg = max(-self.max_deg, min(self.max_deg, deg))
        self.last_cmd_time = time.monotonic()

    def _cb_cmd_vel(self, msg: Twist):
        deg = math.degrees(float(msg.angular.z))
        self.target_deg = max(-self.max_deg, min(self.max_deg, deg))
        self.last_cmd_time = time.monotonic()

    def _cb_lowlevel_fault(self, msg: Bool):
        self.lowlevel_fault = bool(msg.data)

    def _cb_estop(self, msg: Bool):
        self.estop_active = bool(msg.data)

    # ================= TX =================
    def _tx_loop(self):
        now = time.monotonic()

        cmd_stale = (self.last_cmd_time is None
                     or now - self.last_cmd_time > self.cmd_to)
        safety_blocked = (
            self.require_lowlevel_safety
            and (self.lowlevel_fault or self.estop_active)
        )
        safe_required = cmd_stale or safety_blocked

        if safe_required and not self._fault_logged:
            self.get_logger().warn(
                'steering safe state: '
                f'cmd_stale={cmd_stale}, '
                f'lowlevel_fault={self.lowlevel_fault}, '
                f'estop={self.estop_active}')
            self._fault_logged = True
        elif not safe_required:
            self._fault_logged = False

        if safe_required and self.disable_on_fault:
            if self.motor_enabled:
                self._send_disable()
            self.pub_fault.publish(Bool(data=True))
            return

        # re-enable เป็นระยะเฉพาะตอนใช้งานปกติ หรือเมื่อเลือก safe แบบ center
        # เพื่อให้ position frame ยังควบคุมพวงมาลัยกลับไปมุมปลอดภัยได้
        if not self.motor_enabled or now - self.last_enable_time >= self.en_period:
            self._send_enable()

        cmd_deg = self.safe_deg if safe_required else self.target_deg
        counts = self._deg_to_counts(cmd_deg)
        self._send_position(counts)

        # fault publish
        hb_lost = (self.last_hb_time is None
                   or now - self.last_hb_time > 1.0)
        self.pub_fault.publish(Bool(data=bool(safe_required or hb_lost or self.error_code)))

    def _deg_to_counts(self, deg: float) -> int:
        scale = self.scale_pos if deg >= 0.0 else self.scale_neg
        counts = int(round(deg * scale))
        return max(-15000, min(15000, counts))

    def _send_enable(self):
        self._tx(bytes.fromhex('230D200100000000'))
        self.last_enable_time = time.monotonic()
        self.motor_enabled = True

    def _send_disable(self):
        self._tx(bytes.fromhex('230C200100000000'))
        self.motor_enabled = False

    def _send_position(self, counts: int):
        u = counts & 0xFFFFFFFF
        lo, hi = u & 0xFFFF, (u >> 16) & 0xFFFF
        data = bytes([0x23, 0x02, 0x20, 0x01,
                      (lo >> 8) & 0xFF, lo & 0xFF,
                      (hi >> 8) & 0xFF, hi & 0xFF])
        self._tx(data)

    def _tx(self, data: bytes):
        try:
            self.bus.send(can.Message(arbitration_id=self.id_cmd, data=data,
                                      is_extended_id=True))
        except can.CanError as e:
            self.get_logger().error(f'CAN TX failed: {e}',
                                    throttle_duration_sec=1.0)

    # ================= RX =================
    def _rx_loop(self):
        while not self._rx_stop.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None or not msg.is_extended_id:
                continue
            if msg.arbitration_id != self.id_hb or len(msg.data) < 8:
                continue

            d = bytes(msg.data)
            raw_angle  = struct.unpack_from('>H', d, 0)[0]   # big-endian
            self.error_code = struct.unpack_from('>H', d, 6)[0]
            self.last_hb_time = time.monotonic()

            # cumulative angle wraps ที่ 65535 -> integrate delta แบบ wrap-aware
            if self.hb_prev_raw is not None:
                delta = raw_angle - self.hb_prev_raw
                if delta > 32767:
                    delta -= 65536
                elif delta < -32768:
                    delta += 65536
                # heartbeat หน่วยเป็น "องศาของแกนมอเตอร์" -> counts
                self.motor_counts += delta * HB_DEG_PER_LSB \
                    * (COUNTS_PER_MOTOR_REV / 360.0)
            self.hb_prev_raw = raw_angle

            # counts -> มุมเลี้ยวล้อ (ใช้ scale ฝั่งเดียวกับทิศของ counts)
            scale = self.scale_pos if self.motor_counts >= 0 else self.scale_neg
            steer_deg = self.motor_counts / scale
            self.pub_angle.publish(Float64(data=math.radians(steer_deg)))

            if self.error_code:
                self.get_logger().warn(
                    f'motor error 0x{self.error_code:04X}',
                    throttle_duration_sec=2.0)

    def destroy_node(self):
        self._rx_stop.set()
        self._rx_thread.join(timeout=1.0)
        try:
            self._send_disable()
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

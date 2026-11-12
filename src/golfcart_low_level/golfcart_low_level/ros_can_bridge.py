#!/usr/bin/env python3
"""
ros_can_bridge — supervisor + ROS2 <-> CAN bridge (runs on Jetson)

Flow ต่อ 1 รอบ control loop (50 Hz):
  1) รับ /cmd_vel (Twist) จาก nav/teleop
  2) อ่าน e-stop/fault จาก STM32 heartbeat/status frames -> /e_stop_status
  3) สร้าง condition (e-stop / cmd_vel timeout / board fault)
  4) จากนั้นค่อยสั่ง steering (/steering_cmd), brake (0x130), speed (0x120/0x121)

Manual brake test:
  /brake_cmd (std_msgs/Bool) = true  -> enable=0, speed=0, brake=1
  /brake_cmd (std_msgs/Bool) = false -> return control to /cmd_vel + safety logic

cmd_vel.angular.z = yaw rate (rad/s) จริง -> แปลงเป็นมุมเลี้ยวด้วย bicycle model
  delta = atan(wheelbase * yaw_rate / v)   (มี guard กันหารด้วยศูนย์)

Wire format (ตรงกับ firmware/stm32_motorbrake_car_shield ที่ sync จาก
AJYui_MotorBrakeModule-Dev_dataloggerShield/MotorBrake_Firmware_Car_Shield):
  TX 0x120  int16 LE, 0.01 m/s/LSB   target speed
  TX 0x121  byte0: 1=enable PID      speed enable
  RX 0x122  8 bytes                   measured/target speed + flags + seq
  RX 0x123  8 bytes                   speed diagnostics
  TX 0x130  byte0: 1=brake 0=release brake command
  TX 0x132  2 x float32 LE            brake release/start angle, brake engage/stop angle
  RX 0x131  [0..3] float32 LE current_mA, [4] relay_active,
            [5] bit0 watchdog/fault, bit1 PC13 e-stop, [6..7] uint16 seq
  RX 0x133  2 x float32 LE            servo start/stop angle status
  RX 0x134  uint8                     current brake servo target angle_deg

Safety rules (diagram V2):
  1) e-stop (จาก 0x131)   -> enable=0, speed=0, brake=1
  2) no /cmd_vel or /cmd_vel timeout -> wait, enable=0, speed=0, brake=0
  3) board fault/heartbeat -> enable=0, speed=0, brake=1
  4) /brake_cmd true       -> enable=0, speed=0, brake=1
  5) normal moving         -> brake=0, enable=1, speed=linear.x, /steering_cmd
  6) normal zero command   -> enable=0, speed=0, brake only while measured speed is not zero

Before running:
  sudo ip link set can0 up type can bitrate 250000
"""

import math
import struct
import threading

import can
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Float32, UInt8, UInt16, String

CAN_ID_SPEED_CMD    = 0x120
CAN_ID_SPEED_ENABLE = 0x121
CAN_ID_SPEED_STATUS = 0x122
CAN_ID_SPEED_DIAG   = 0x123
CAN_ID_BRAKE_CMD    = 0x130
CAN_ID_BRAKE_STATUS = 0x131
CAN_ID_BRAKE_SERVO_CMD = 0x132
CAN_ID_BRAKE_SERVO_STATUS = 0x133
CAN_ID_BRAKE_ANGLE = 0x134

SPEED_SCALE = 100.0          # int16 raw = m/s * 100
SPEED_FLAG_ENABLED   = 0x01
SPEED_FLAG_FWD_CMD   = 0x02
SPEED_FLAG_REV_CMD   = 0x04
SPEED_FLAG_PEDAL     = 0x08
SPEED_FLAG_SENSOR_OK = 0x10
SPEED_FLAG_TIMEOUT   = 0x20
SPEED_FLAG_ESTOP     = 0x40
SPEED_FLAG_FAULT     = 0x80


class RosCanBridge(Node):
    def __init__(self):
        super().__init__('ros_can_bridge')

        # ---------- parameters ----------
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 250000)      # SocketCAN must be brought up with this
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)
        self.declare_parameter('board_timeout_s', 0.5)     # 0x131 มาทุก 20 ms
        self.declare_parameter('brake_cmd_can_id', CAN_ID_BRAKE_CMD)
        self.declare_parameter('brake_status_can_id', CAN_ID_BRAKE_STATUS)
        self.declare_parameter('brake_servo_cmd_can_id', CAN_ID_BRAKE_SERVO_CMD)
        self.declare_parameter('brake_servo_start_deg', 180.0)
        self.declare_parameter('brake_servo_stop_deg', 110.0)
        self.declare_parameter('wheelbase_m', 1.65)        # <- วัดจากรถจริง
        self.declare_parameter('max_speed_mps', 5.0)
        self.declare_parameter('max_steering_deg', 50.0)
        # yaw_rate: cmd_vel.angular.z is rad/s.
        # steering_angle: cmd_vel.angular.z is direct road-wheel steering angle in rad.
        self.declare_parameter('cmd_vel_steering_mode', 'yaw_rate')
        # ต่ำกว่าความเร็วนี้จะไม่คำนวณ atan (กันหารด้วยศูนย์ / มุมระเบิดที่ v ต่ำ)
        self.declare_parameter('min_speed_for_steering_mps', 0.1)
        # true = zero command always holds brake.
        # false = zero command brakes only until measured speed reaches zero.
        self.declare_parameter('brake_on_zero_cmd', False)
        self.declare_parameter('zero_cmd_brake_release_speed_mps', 0.03)

        ch          = self.get_parameter('can_channel').value
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        rate        = self.get_parameter('control_rate_hz').value
        self.cmd_to = self.get_parameter('cmd_vel_timeout_s').value
        self.brd_to = self.get_parameter('board_timeout_s').value
        self.brake_cmd_can_id = int(self.get_parameter('brake_cmd_can_id').value)
        self.brake_status_can_id = int(self.get_parameter('brake_status_can_id').value)
        self.brake_servo_cmd_can_id = int(self.get_parameter('brake_servo_cmd_can_id').value)
        self.servo_start_deg = self._clamp_angle(self.get_parameter('brake_servo_start_deg').value)
        self.servo_stop_deg = self._clamp_angle(self.get_parameter('brake_servo_stop_deg').value)
        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.max_speed = self.get_parameter('max_speed_mps').value
        self.max_steer = math.radians(self.get_parameter('max_steering_deg').value)
        self.steering_mode = str(self.get_parameter('cmd_vel_steering_mode').value)
        self.min_v     = self.get_parameter('min_speed_for_steering_mps').value
        self.brake_on_zero_cmd = bool(self.get_parameter('brake_on_zero_cmd').value)
        self.zero_brake_release_speed = abs(
            float(self.get_parameter('zero_cmd_brake_release_speed_mps').value)
        )

        # ---------- CAN ----------
        self.bus = can.Bus(channel=ch, interface='socketcan')
        self._rx_stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

        # ---------- state ----------
        self.cmd = Twist()
        self.last_cmd_stamp = None          # None = ยังไม่เคยได้ /cmd_vel
        self.estop_active = False           # ใช้สถานะจริงจาก 0x131/0x122; board_lost ยังเป็น fail-safe ตอนเริ่ม
        self.brake_estop_active = False
        self.speed_estop_active = False
        self.speed_fault_active = False
        self.last_board_stamp = None
        self.last_steer = 0.0
        self.measured_speed = 0.0
        self.target_speed = 0.0
        self.last_brake_angle_cmd_deg = self.servo_stop_deg
        self.manual_brake_active = False
        self._last_fault_reason = None

        # ---------- ROS I/O ----------
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(Bool, '/brake_cmd', self._cb_brake_cmd, 10)
        self.create_subscription(Float32, '/servo_command', self._cb_brake_angle, 10)
        self.create_subscription(Float32, '/brake_angle_cmd', self._cb_brake_angle, 10)
        self.create_subscription(Float32, '/brake_servo_start_cmd', self._cb_servo_start, 10)
        self.create_subscription(Float32, '/brake_servo_stop_cmd', self._cb_servo_stop, 10)

        self.pub_steer_cmd   = self.create_publisher(Float64, '/steering_cmd', 10)
        self.pub_estop       = self.create_publisher(Bool,   '/e_stop_status', 10)
        self.pub_speed_meas  = self.create_publisher(Float64, '/speed_status', 10)
        self.pub_speed_target = self.create_publisher(Float64, '/speed_target', 10)
        self.pub_speed_flags = self.create_publisher(UInt8, '/speed_status_flags', 10)
        self.pub_speed_fault_code = self.create_publisher(UInt8, '/speed_fault_code', 10)
        self.pub_speed_seq = self.create_publisher(UInt16, '/speed_sequence', 10)
        self.pub_speed_diag = self.create_publisher(String, '/speed_diagnostics', 10)
        self.pub_brake_state = self.create_publisher(UInt8,  '/brake_status', 10)
        self.pub_brake_cur   = self.create_publisher(Float32, '/brake_current', 10)
        self.pub_brake_seq   = self.create_publisher(UInt16, '/brake_sequence', 10)
        self.pub_brake_angle = self.create_publisher(Float32, '/brake_angle', 10)
        self.pub_servo_status = self.create_publisher(String, '/servo_status', 10)
        self.pub_fault       = self.create_publisher(Bool,   '/lowlevel_fault', 10)
        self.pub_fault_reason = self.create_publisher(String, '/lowlevel_fault_reason', 10)

        self.create_timer(1.0 / rate, self._control_loop)
        self._rx_thread.start()
        self.get_logger().info(
            f'ros_can_bridge up on {ch}; expected CAN bitrate {self.can_bitrate} bps '
            f'(cmd_vel_steering_mode={self.steering_mode})')

    # ================= ROS callbacks =================
    def _cb_cmd_vel(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_stamp = self.get_clock().now()

    def _cb_brake_cmd(self, msg: Bool):
        active = bool(msg.data)
        if active != self.manual_brake_active:
            state = 'active' if active else 'released'
            self.get_logger().info(f'manual /brake_cmd override {state}')
        self.manual_brake_active = active

    def _cb_brake_angle(self, msg: Float32):
        angle = self._clamp_angle(msg.data)
        self.last_brake_angle_cmd_deg = angle
        self.servo_stop_deg = angle
        self._send_servo_angles()

    def _cb_servo_start(self, msg: Float32):
        self.servo_start_deg = self._clamp_angle(msg.data)
        self._send_servo_angles()

    def _cb_servo_stop(self, msg: Float32):
        self.servo_stop_deg = self._clamp_angle(msg.data)
        self.last_brake_angle_cmd_deg = self.servo_stop_deg
        self._send_servo_angles()

    # ================= main control loop =================
    def _control_loop(self):
        now = self.get_clock().now()

        # ---- step 1-3: สร้าง condition ก่อน ----
        waiting_for_cmd = self.last_cmd_stamp is None
        cmd_timeout = (
            not waiting_for_cmd
            and (now - self.last_cmd_stamp).nanoseconds * 1e-9 > self.cmd_to
        )
        board_lost = (
            self.last_board_stamp is None
            or (now - self.last_board_stamp).nanoseconds * 1e-9 > self.brd_to
        )

        no_cmd_available = waiting_for_cmd or cmd_timeout

        if self.estop_active:
            self.pub_fault.publish(Bool(data=True))
            self._publish_fault_reason(False, board_lost)
            self._send_enable(0)
            self._send_speed(0.0)
            self._send_brake(1)
            return

        if no_cmd_available:
            # Startup/idle wait: no navigation command has arrived yet.
            # Do not engage the brake just because the command source is not up
            # or has stopped publishing. A real e-stop still overrides above.
            reason = 'waiting_for_cmd' if waiting_for_cmd else 'cmd_vel_timeout_waiting'
            self.pub_fault.publish(Bool(data=False))
            self.pub_fault_reason.publish(String(data=reason))
            self._send_enable(0)
            self._send_speed(0.0)
            self._send_brake(0)
            return

        stop_required = self.speed_fault_active or board_lost
        self.pub_fault.publish(Bool(data=bool(stop_required)))
        self._publish_fault_reason(cmd_timeout, board_lost)

        # ---- step 4: สั่ง actuator ตาม condition ----
        if stop_required:
            # rules 1-3: หยุด + เบรก, steering ค้างมุมเดิม (ไม่ publish)
            self._send_enable(0)
            self._send_speed(0.0)
            self._send_brake(1)
            return

        if self.manual_brake_active:
            # Manual brake override for bench/hardware tests. Safety faults above
            # still win; false simply returns control to the normal /cmd_vel path.
            self._send_enable(0)
            self._send_speed(0.0)
            self._send_brake(1)
            return

        v = float(self.cmd.linear.x)
        v = max(-self.max_speed, min(self.max_speed, v))

        if abs(v) > 1e-3:
            self._send_brake(0)
            self._send_enable(1)
            self._send_speed(v)
        else:
            # ผู้ใช้สั่งหยุด/idle (linear.x = 0): ปล่อยคันเร่งเสมอ.
            # ถ้ารถยังเคลื่อนที่ให้เบรกจนกว่าความเร็วจริงจะลงใกล้ศูนย์.
            # ตอนเริ่มระบบที่รถหยุดนิ่งอยู่แล้วจึงไม่สั่งเบรกค้าง.
            self._send_enable(0)
            self._send_speed(0.0)
            rolling = abs(self.measured_speed) > self.zero_brake_release_speed
            self._send_brake(1 if self.brake_on_zero_cmd or rolling else 0)

        steer = self._compute_steering(v)
        self.last_steer = steer
        self.pub_steer_cmd.publish(Float64(data=steer))

    def _publish_fault_reason(self, cmd_timeout: bool, board_lost: bool):
        reasons = []
        if cmd_timeout:
            reasons.append('cmd_vel_timeout')
        if board_lost:
            reasons.append('brake_board_0x131_timeout')
        if self.estop_active:
            reasons.append('e_stop_active')
        if self.speed_fault_active:
            reasons.append('speed_fault')

        reason = ','.join(reasons) if reasons else 'clear'
        self.pub_fault_reason.publish(String(data=reason))
        if reason != self._last_fault_reason:
            if reason == 'clear':
                self.get_logger().info(f'lowlevel_fault_reason={reason}')
            else:
                self.get_logger().warn(f'lowlevel_fault_reason={reason}')
            self._last_fault_reason = reason

    def _compute_steering(self, v: float) -> float:
        if self.steering_mode == 'steering_angle':
            steer = float(self.cmd.angular.z)
            return max(-self.max_steer, min(self.max_steer, steer))

        if self.steering_mode != 'yaw_rate':
            self.get_logger().warn(
                f'unknown cmd_vel_steering_mode={self.steering_mode}; using yaw_rate',
                throttle_duration_sec=2.0)

        """bicycle model: delta = atan(L * yaw_rate / v) พร้อม guard หารด้วยศูนย์"""
        yaw_rate = float(self.cmd.angular.z)

        if abs(v) < self.min_v:
            # ความเร็วต่ำ/ศูนย์: ห้ามหาร
            if abs(yaw_rate) < 1e-3:
                return 0.0              # ไม่ขยับ + ไม่สั่งเลี้ยว -> คืนพวงมาลัยตรง
            return self.last_steer      # อยากเลี้ยวแต่รถแทบไม่วิ่ง -> ค้างมุมเดิม

        steer = math.atan(self.wheelbase * yaw_rate / v)
        return max(-self.max_steer, min(self.max_steer, steer))

    # ================= CAN TX =================
    def _tx(self, can_id: int, data: bytes):
        try:
            self.bus.send(can.Message(arbitration_id=can_id, data=data,
                                      is_extended_id=False))
        except can.CanError as e:
            self.get_logger().error(f'CAN TX 0x{can_id:03X} failed: {e}',
                                    throttle_duration_sec=1.0)

    def _send_speed(self, mps: float):
        raw = int(round(mps * SPEED_SCALE))
        raw = max(-32768, min(32767, raw))
        self._tx(CAN_ID_SPEED_CMD, struct.pack('<h', raw))

    def _send_enable(self, en: int):
        self._tx(CAN_ID_SPEED_ENABLE, bytes([1 if en else 0]))

    def _send_brake(self, b: int):
        self._tx(self.brake_cmd_can_id, bytes([1 if b else 0]))

    @staticmethod
    def _clamp_angle(angle_deg: float) -> float:
        return max(0.0, min(180.0, float(angle_deg)))

    def _send_servo_angles(self):
        start = self._clamp_angle(self.servo_start_deg)
        stop = self._clamp_angle(self.servo_stop_deg)
        self.servo_start_deg = start
        self.servo_stop_deg = stop
        self._tx(self.brake_servo_cmd_can_id, struct.pack('<ff', start, stop))

    def _update_estop_state(self):
        self.estop_active = bool(self.brake_estop_active or self.speed_estop_active)

    # ================= CAN RX (thread) =================
    def _rx_loop(self):
        while not self._rx_stop.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None or msg.is_extended_id:
                continue                       # extended = KEYA, ปล่อยให้ steering_node

            if msg.arbitration_id == CAN_ID_SPEED_STATUS and len(msg.data) >= 8:
                d = bytes(msg.data)
                measured_raw, target_raw = struct.unpack_from('<hh', d, 0)
                flags = d[4]
                fault_code = d[5]
                sequence = struct.unpack_from('<H', d, 6)[0]

                self.measured_speed = measured_raw / SPEED_SCALE
                self.target_speed = target_raw / SPEED_SCALE
                self.speed_estop_active = bool(flags & SPEED_FLAG_ESTOP)
                self.speed_fault_active = bool(flags & (SPEED_FLAG_TIMEOUT | SPEED_FLAG_FAULT)) or fault_code != 0
                self._update_estop_state()

                self.pub_estop.publish(Bool(data=self.estop_active))
                self.pub_speed_meas.publish(Float64(data=self.measured_speed))
                self.pub_speed_target.publish(Float64(data=self.target_speed))
                self.pub_speed_flags.publish(UInt8(data=flags))
                self.pub_speed_fault_code.publish(UInt8(data=fault_code))
                self.pub_speed_seq.publish(UInt16(data=sequence))

            elif msg.arbitration_id == CAN_ID_SPEED_DIAG and len(msg.data) >= 8:
                d = bytes(msg.data)
                input_flags = d[0]
                output_flags = d[1]
                mcor_in_mv = struct.unpack_from('<H', d, 2)[0]
                mcor_out_mv = struct.unpack_from('<H', d, 4)[0]
                sensor_hz = struct.unpack_from('<H', d, 6)[0]
                self.pub_speed_diag.publish(String(data=(
                    f'fwd_input={bool(input_flags & 0x01)}, '
                    f'rev_input={bool(input_flags & 0x02)}, '
                    f'pedal_input={bool(input_flags & 0x04)}, '
                    f'fwd_output={bool(output_flags & 0x01)}, '
                    f'rev_output={bool(output_flags & 0x02)}, '
                    f'pedal_output={bool(output_flags & 0x04)}, '
                    f'mode_relay={bool(output_flags & 0x08)}, '
                    f'mcor_input_v={mcor_in_mv / 1000.0:.3f}, '
                    f'mcor_output_v={mcor_out_mv / 1000.0:.3f}, '
                    f'speed_sensor_hz={float(sensor_hz):.1f}'
                )))

            elif msg.arbitration_id == self.brake_status_can_id and len(msg.data) >= 8:
                d = bytes(msg.data)
                current_ma = struct.unpack_from('<f', d, 0)[0]
                relay      = d[4]
                status_bits = int(d[5])
                watchdog   = bool(status_bits & 0x01)
                pc13_estop = bool(status_bits & 0x02)
                sequence   = struct.unpack_from('<H', d, 6)[0]
                estop      = watchdog or pc13_estop
                self.last_board_stamp = self.get_clock().now()

                if estop and not self.estop_active:
                    self.get_logger().warn('Brake watchdog/e-stop from brake board (0x131)')
                self.brake_estop_active = estop
                self._update_estop_state()

                self.pub_estop.publish(Bool(data=self.estop_active))
                self.pub_brake_state.publish(UInt8(data=relay))
                self.pub_brake_cur.publish(Float32(data=current_ma))
                self.pub_brake_seq.publish(UInt16(data=sequence))

            elif msg.arbitration_id == CAN_ID_BRAKE_SERVO_STATUS and len(msg.data) >= 8:
                start, stop = struct.unpack_from('<ff', bytes(msg.data), 0)
                self.servo_start_deg = self._clamp_angle(start)
                self.servo_stop_deg = self._clamp_angle(stop)
                self.pub_servo_status.publish(String(data=(
                    f'start_deg={self.servo_start_deg:.1f}, '
                    f'stop_deg={self.servo_stop_deg:.1f}'
                )))

            elif msg.arbitration_id == CAN_ID_BRAKE_ANGLE and len(msg.data) >= 1:
                self.pub_brake_angle.publish(Float32(data=float(msg.data[0])))

    def destroy_node(self):
        self._rx_stop.set()
        self._rx_thread.join(timeout=1.0)
        try:
            # fail-safe ก่อนปิด node
            self._send_enable(0)
            self._send_speed(0.0)
            #self._send_brake(1)   ในกรณีที่ ctrl-c คำสั่่ง ros2 launch golfcart_low_level low_level.launch.py ---> ระบบเบรกทำงาน
            self._send_brake(0)    #ในกรณีที่ ctrl-c คำสั่่ง ros2 launch golfcart_low_level low_level.launch.py ---> ระบบเบรกไม่ทำงาน
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

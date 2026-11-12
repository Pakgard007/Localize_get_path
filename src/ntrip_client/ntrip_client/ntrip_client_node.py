"""NTRIP client: pulls RTCM3 corrections from a caster and publishes them on /rtcm.

The ublox_gps node subscribes to /rtcm (rtcm_msgs/Message) and forwards each
message to the receiver, so running this node next to it gives the F9P RTK.

VRS mountpoints only start streaming once the client sends its position, so a
GGA sentence is pushed back up the same socket periodically. It is taken from
the receiver's own NMEA output when available, otherwise synthesized from the
NavSatFix.
"""

import base64
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message as RtcmMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus

RTCM3_PREAMBLE = 0xD3


def nmea_checksum(body: str) -> str:
    csum = 0
    for ch in body:
        csum ^= ord(ch)
    return f"{csum:02X}"


def build_gga(fix: NavSatFix) -> str:
    """Synthesize a $GPGGA sentence from a NavSatFix (fallback when no NMEA)."""
    now = time.gmtime()
    utc = f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.00"

    lat = abs(fix.latitude)
    lat_deg = int(lat)
    lat_min = (lat - lat_deg) * 60.0
    lat_hemi = "N" if fix.latitude >= 0 else "S"

    lon = abs(fix.longitude)
    lon_deg = int(lon)
    lon_min = (lon - lon_deg) * 60.0
    lon_hemi = "E" if fix.longitude >= 0 else "W"

    quality = 1 if fix.status.status >= NavSatStatus.STATUS_FIX else 0

    body = (
        f"GPGGA,{utc},"
        f"{lat_deg:02d}{lat_min:09.6f},{lat_hemi},"
        f"{lon_deg:03d}{lon_min:09.6f},{lon_hemi},"
        f"{quality},12,1.0,{fix.altitude:.2f},M,0.0,M,,"
    )
    return f"${body}*{nmea_checksum(body)}"


class NtripClient(Node):

    def __init__(self):
        super().__init__('ntrip_client')

        self.host = self.declare_parameter('host', '110.78.0.54').value
        self.port = self.declare_parameter('port', 2116).value
        self.mountpoint = self.declare_parameter('mountpoint', 'VRS_RTCM32').value
        self.username = self.declare_parameter('username', '').value
        self.password = self.declare_parameter('password', '').value

        self.gga_interval = self.declare_parameter('gga_interval', 0.2).value
        self.connect_timeout = self.declare_parameter('connect_timeout', 10.0).value
        self.retry_interval = self.declare_parameter('retry_interval', 5.0).value
        rtcm_topic = self.declare_parameter('rtcm_topic', '/rtcm').value
        nmea_topic = self.declare_parameter('nmea_topic', '/nmea').value
        fix_topic = self.declare_parameter('fix_topic', '/ublox_gps_node/fix').value

        # Seed position: the caster needs a GGA before it sends anything, and a
        # GGA needs a fix — which needs corrections. Seeding a rough position
        # breaks that loop, so RTCM flows while the receiver is still searching.
        seed_lat = self.declare_parameter('seed_latitude', 0.0).value
        seed_lon = self.declare_parameter('seed_longitude', 0.0).value
        seed_alt = self.declare_parameter('seed_altitude', 0.0).value

        # Raw NMEA from the receiver if we have it, otherwise a position we
        # rebuild a GGA from at send time — the caster drops the stream if the
        # timestamp inside the sentence stops advancing.
        self._raw_gga: str | None = None
        self._fix: NavSatFix | None = None
        self._frames = 0
        self._gga_lock = threading.Lock()

        if seed_lat or seed_lon:
            seed = NavSatFix()
            seed.latitude = seed_lat
            seed.longitude = seed_lon
            seed.altitude = seed_alt
            seed.status.status = NavSatStatus.STATUS_FIX
            self._fix = seed
            self.get_logger().info(
                f"🌱 Seed position {seed_lat:.5f}, {seed_lon:.5f} — "
                "requesting corrections before the first fix"
            )

        self.rtcm_pub = self.create_publisher(RtcmMessage, rtcm_topic, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Sentence, nmea_topic, self.nmea_callback, sensor_qos)
        self.create_subscription(NavSatFix, fix_topic, self.fix_callback, sensor_qos)

        self._thread = threading.Thread(target=self._ntrip_thread, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"📡 NTRIP client → {self.host}:{self.port}/{self.mountpoint}, "
            f"publishing RTCM on {rtcm_topic}"
        )

    # ── GGA sources ───────────────────────────────────────────────────────────

    def nmea_callback(self, msg: Sentence):
        # Raw GGA straight from the receiver beats anything we can rebuild, but
        # only while it holds a position. Before its first fix (and after
        # losing one) the receiver sends "$GNGGA,hhmmss.ss,,,,,0,..." with
        # empty coordinates; forwarding that replaced the seed and the VRS
        # caster went silent. Dropping it makes current_gga() rebuild the last
        # real fix, or the seed, with a fresh timestamp instead.
        sentence = msg.sentence.strip()
        if 'GGA' not in sentence[:7]:
            return
        fields = sentence.split(',')
        has_position = (
            len(fields) > 6 and fields[2] and fields[4] and fields[6] not in ('', '0')
        )
        with self._gga_lock:
            self._raw_gga = sentence if has_position else None

    def fix_callback(self, msg: NavSatFix):
        # STATUS_FIX is 0, so the status field alone does not separate a real
        # fix from an empty one: this driver publishes status 0 with lat/lon
        # 0,0 while the receiver is still searching. Sending that null-island
        # position to a VRS caster makes it stop streaming, so check the
        # coordinates themselves.
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if abs(msg.latitude) < 1e-7 and abs(msg.longitude) < 1e-7:
            return
        with self._gga_lock:
            self._fix = msg

    def current_gga(self) -> str | None:
        """The GGA to send right now, with a fresh timestamp."""
        with self._gga_lock:
            # Raw NMEA wins; the rebuilt one covers receivers with NMEA off.
            if self._raw_gga:
                return self._raw_gga
            return build_gga(self._fix) if self._fix else None

    # ── NTRIP connection ──────────────────────────────────────────────────────

    def _ntrip_thread(self):
        while rclpy.ok():
            sock = None
            try:
                sock = self._open_socket()
                auth = base64.b64encode(
                    f"{self.username}:{self.password}".encode()
                ).decode()
                request = (
                    f"GET /{self.mountpoint} HTTP/1.0\r\n"
                    f"User-Agent: NTRIP ROS2Client/1.0\r\n"
                    f"Authorization: Basic {auth}\r\n\r\n"
                )
                sock.sendall(request.encode())
                self.get_logger().info("✅ NTRIP connected")

                sock.settimeout(0.1)
                buffer = bytearray()
                last_gga = 0.0
                last_report = time.time()
                self._frames = 0

                while rclpy.ok():
                    try:
                        data = sock.recv(4096)
                        if not data:
                            raise ConnectionError("caster closed the connection")
                        buffer.extend(data)
                        self._drain_rtcm(buffer)
                    except socket.timeout:
                        pass

                    now = time.time()
                    if now - last_report > 10.0:
                        self.get_logger().info(
                            f"📶 RTCM {self._frames} frames in the last 10 s"
                            if self._frames
                            else "⚠️  no RTCM from the caster — is the GGA position valid?"
                        )
                        self._frames = 0
                        last_report = now

                    if now - last_gga > self.gga_interval:
                        gga = self.current_gga()
                        if gga:
                            sock.sendall((gga + "\r\n").encode())
                            last_gga = now

            except Exception as e:  # noqa: BLE001 - keep the client alive
                self.get_logger().error(f"❌ NTRIP error: {e}")
            finally:
                if sock:
                    try:
                        sock.close()
                    except OSError:
                        pass
                time.sleep(self.retry_interval)

    def _open_socket(self):
        last_error = None
        for family, socktype, proto, _canon, addr in socket.getaddrinfo(
            self.host, self.port, type=socket.SOCK_STREAM
        ):
            sock = socket.socket(family, socktype, proto)
            sock.settimeout(self.connect_timeout)
            try:
                sock.connect(addr)
                return sock
            except OSError as e:
                last_error = e
                sock.close()
        raise last_error or OSError(f"Cannot resolve NTRIP host {self.host}")

    # ── RTCM3 framing ─────────────────────────────────────────────────────────

    def _drain_rtcm(self, buffer: bytearray):
        """Publish every complete RTCM3 frame sitting in the buffer.

        Frame: 0xD3 | 6 reserved bits + 10-bit length | payload | 3-byte CRC.
        The caster's HTTP response header is skipped for free, since anything
        before the first preamble is discarded.
        """
        while True:
            start = buffer.find(bytes([RTCM3_PREAMBLE]))
            if start < 0:
                # No frame start in sight; keep the tail in case of a split.
                del buffer[:-1]
                return
            if start > 0:
                del buffer[:start]
            if len(buffer) < 3:
                return

            length = ((buffer[1] & 0x03) << 8) | buffer[2]
            frame_len = 3 + length + 3
            if len(buffer) < frame_len:
                return

            frame = bytes(buffer[:frame_len])
            del buffer[:frame_len]

            msg = RtcmMessage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'
            msg.message = list(frame)
            self.rtcm_pub.publish(msg)
            self._frames += 1


def main(args=None):
    rclpy.init(args=args)
    node = NtripClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

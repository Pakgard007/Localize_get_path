"""Launch the ublox gps node with the C102-F9R (ZED-F9R) config.

zed_f9r.yaml runs the driver with config_on_startup: false (the driver's
legacy CFG-TMODE3 is NACKed by F9 firmware), so before starting the node this
launch writes the UBX message/rate setup itself with UBX-CFG-VALSET (RAM layer
only, so there is no flash wear from launching repeatedly).
"""

import os
import struct
import time

import ament_index_python.packages
import launch
import launch_ros.actions
import serial
import yaml

# (key id, value size in bytes, value) -- ZED-F9R HPS 1.2x/1.3x interface description
F9R_CONFIG = [
    (0x30210001, 2, 200),  # CFG-RATE-MEAS: 200 ms => 5 Hz (keep zed_f9r.yaml rate in sync)
    (0x30210002, 2, 1),    # CFG-RATE-NAV: 1 nav solution per measurement
    (0x20110021, 1, 4),    # CFG-NAVSPG-DYNMODEL: 4 = automotive
    (0x10080001, 1, 1),    # CFG-SFCORE-USE_SF: sensor fusion on
    (0x10060027, 1, 1),    # CFG-SFIMU-AUTO_MNTALG_ENA: auto IMU mount alignment
    (0x10770004, 1, 1),    # CFG-USBINPROT-RTCM3X: accept RTCM from ntrip_client
    (0x10780001, 1, 1),    # CFG-USBOUTPROT-UBX
    (0x20910009, 1, 1),    # CFG-MSGOUT-UBX_NAV_PVT_USB -> /fix, /fix_velocity
    (0x20910036, 1, 1),    # CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB
    (0x2091001d, 1, 1),    # CFG-MSGOUT-UBX_NAV_STATUS_USB
    (0x20910018, 1, 5),    # CFG-MSGOUT-UBX_NAV_SAT_USB (1 Hz)
    (0x20910108, 1, 5),    # CFG-MSGOUT-UBX_ESF_STATUS_USB (1 Hz, fusion state)
    (0x2091026b, 1, 5),    # CFG-MSGOUT-UBX_RXM_RTCM_USB (1 Hz, correction input)
]

# Dead-reckoning output. Without these the F9R fuses internally but never says
# so on the wire: the driver's AdrUdrProduct subscribes to NAV-ATT/ESF-INS/
# ESF-MEAS and publishes navatt, esfins and imu_meas (sensor_msgs/Imu) only if
# the receiver actually emits them.
#
# These key ids are NOT verified against an HPS interface description here, so
# they are sent one VALSET at a time rather than appended to F9R_CONFIG: a
# single bad key NAKs the WHOLE combined VALSET, which would take NAV-PVT --
# and therefore /fix -- down with it. Sent individually, a wrong id costs only
# its own message and says so in the log.
F9R_DR_CONFIG = [
    (0x20910022, 1, 1, 'NAV_ATT'),    # attitude: roll/pitch/heading + accuracies
    (0x20910117, 1, 1, 'ESF_INS'),    # fusion-compensated angular rate / accel
    (0x20910279, 1, 1, 'ESF_MEAS'),   # raw sensor measurements -> imu_meas
]
LAYER_RAM = 0x01
DEVICE_WAIT = 10.0  # [s] to wait for the USB device to appear
SIZE_FMT = {1: '<B', 2: '<H', 4: '<I'}


def ubx(cls, mid, payload):
    body = struct.pack('<BBH', cls, mid, len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes([ck_a, ck_b])


def wait_ack(port, cls, mid, timeout=2.0):
    """Return True on ACK-ACK, False on ACK-NAK/timeout for (cls, mid)."""
    buf = b''
    end = time.time() + timeout
    while time.time() < end:
        buf += port.read(port.in_waiting or 1)
        i = buf.find(b'\xb5\x62\x05')
        while i != -1 and len(buf) >= i + 10:
            if buf[i + 6] == cls and buf[i + 7] == mid:
                return buf[i + 3] == 0x01
            i = buf.find(b'\xb5\x62\x05', i + 1)
    return False


def configure_f9r(context, params):
    with open(params) as f:
        device = yaml.safe_load(f)['ublox_gps_node']['ros__parameters']['device']
    log = launch.logging.get_logger('zed_f9r_setup')

    end = time.time() + DEVICE_WAIT
    while not os.path.exists(device) and time.time() < end:
        time.sleep(0.5)
    if not os.path.exists(device):
        log.error(f'{device} not found -- is the C102-F9R plugged in (check dmesg)?')
        return []

    payload = struct.pack('<BBxx', 0, LAYER_RAM)
    for key, size, value in F9R_CONFIG:
        payload += struct.pack('<I', key) + struct.pack(SIZE_FMT[size], value)
    try:
        with serial.Serial(device, 38400, timeout=0.1) as port:
            port.reset_input_buffer()
            port.write(ubx(0x06, 0x8A, payload))
            if wait_ack(port, 0x06, 0x8A):
                log.info(f'{device}: ZED-F9R configured (UBX NAV-PVT @ 5 Hz)')
            else:
                log.error(f'{device}: CFG-VALSET NAKed or unanswered; '
                          'fix/fix_velocity may not publish')

            for key, size, value, name in F9R_DR_CONFIG:
                port.reset_input_buffer()
                port.write(ubx(0x06, 0x8A, struct.pack('<BBxx', 0, LAYER_RAM) +
                               struct.pack('<I', key) +
                               struct.pack(SIZE_FMT[size], value)))
                if wait_ack(port, 0x06, 0x8A):
                    log.info(f'{device}: {name} output enabled')
                else:
                    log.warning(
                        f'{device}: NAKed CFG-MSGOUT-UBX_{name}_USB '
                        f'(key 0x{key:08x}) -- check the id against the HPS '
                        'interface description; the rest of the setup stands')
    except serial.SerialException as e:
        log.error(f'{device}: {e}')
    return []


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')
    params = os.path.join(config_directory, 'zed_f9r.yaml')
    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[params])

    return launch.LaunchDescription([launch.actions.OpaqueFunction(
                                         function=configure_f9r,
                                         args=[params]),
                                     ublox_gps_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])

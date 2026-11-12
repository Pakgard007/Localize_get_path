#!/usr/bin/env bash
# start_cart.sh — bring the CAN bus up cleanly, get the STM32 talking, then
# start golfcart_low_level.
#
#   bash ~/2Localize_get_path/start_cart.sh
#
# Why the order matters: the STM32 firmware has no CAN bus-off recovery. If it
# runs while nothing on the bus ACKs its frames (can0 down, BUS-OFF or
# listen-only), it goes silent until it is power-cycled. The STM32 is the only
# node that ACKs, so ros_can_bridge's 50 Hz TX then drives the USB2CAN adapter
# into BUS-OFF as well ("CAN TX ... failed: Transmit buffer full"), and a later
# STM32 power-cycle cannot help because the adapter can no longer ACK it.
# So: reset can0 with nothing transmitting, power-cycle the STM32 while can0 is
# ready to ACK, wait for its 0x131 status frame, and only then start the nodes.

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IF=can0
BITRATE=250000
WAIT_S=90
# Anchored on the python3 interpreter so shells whose command line merely
# mentions these names are never matched.
PY='^[^ ]*python3[^ ]* [^ ]*'
NODES="${PY}golfcart_low_level/(ros_can_bridge|steering_node)"

stop_running_launch() {
  pgrep -f "$NODES" >/dev/null || return 0
  echo ">> พบ low_level ที่รันค้างอยู่ กำลังหยุดให้..."
  pkill -INT -f "${PY}ros2 launch golfcart_low_level" 2>/dev/null
  for _ in $(seq 1 10); do
    pgrep -f "$NODES" >/dev/null || return 0
    sleep 0.5
  done
  pkill -TERM -f "${PY}golfcart_low_level/(ros_can_bridge|steering_node|odom_bridge)" 2>/dev/null
  sleep 1
}

# Returns 0 as soon as one 0x131 (STM32 brake status) frame arrives.
wait_for_stm32() {
  timeout "$1" candump -n 1 "$IF,131:7FF" >/dev/null 2>&1 &
  local pid=$! i
  for ((i = $1; i > 0; i--)); do
    kill -0 "$pid" 2>/dev/null || break
    printf '\r   รอสัญญาณจาก STM32 ... %3d s ' "$i"
    sleep 1
  done
  printf '\r%45s\r' ''
  wait "$pid"
}

if ! ip link show "$IF" >/dev/null 2>&1; then
  echo "!! ไม่พบ $IF — เสียบอะแดปเตอร์ USB2CAN หรือยัง?"
  exit 1
fi

stop_running_launch

echo ">> 1/3 รีเซ็ต $IF (อาจต้องใส่รหัส sudo)"
sudo ip link set "$IF" down
if ! sudo ip link set "$IF" up type can bitrate "$BITRATE"; then
  echo "!! ตั้งค่า $IF ไม่สำเร็จ"
  exit 1
fi
sudo ip link set "$IF" txqueuelen 1000

echo ">> 2/3 เช็กว่า STM32 ส่งสัญญาณอยู่ไหม"
if ! wait_for_stm32 2; then
  echo
  echo "   ==================================================="
  echo "   >>>  ปิดไฟบอร์ด STM32  รอ 3 วินาที  แล้วเปิดใหม่  <<<"
  echo "   ==================================================="
  echo "   ทำได้เลยตอนนี้ สคริปต์จะรอ ${WAIT_S} วินาที"
  if ! wait_for_stm32 "$WAIT_S"; then
    echo
    echo "!! ยังไม่ได้ยินอะไรจาก STM32 (0x131) — ปัญหาอยู่ที่ฮาร์ดแวร์ ให้เช็ก:"
    echo "   1. บอร์ด STM32 มีไฟเข้า LED ติด"
    echo "   2. สาย CAN จาก USB2CAN ถึงบอร์ด STM32: CANH-CANH, CANL-CANL, เสียบแน่น"
    echo "   3. ปิดไฟ แล้ววัด CANH-CANL ต้องได้ประมาณ 60 ohm"
    exit 1
  fi
fi
echo "   STM32 ตอบแล้ว (ได้รับ 0x131)"

echo ">> 3/3 เริ่ม low_level"
cd "$WS" || exit 1
# shellcheck disable=SC1091
source install/setup.bash
exec ros2 launch golfcart_low_level low_level.launch.py

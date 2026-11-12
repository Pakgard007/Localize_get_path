#!/usr/bin/env bash
# can_check.sh — one-shot CAN bus health check for the golf cart (can0 @ 250 kbps).
#
# Run it with the ROS launch STOPPED:
#   bash can_check.sh
#
# It resets can0 to normal mode (clears listen-only / BUS-OFF / stuck TX queue),
# listens while you power-cycle the STM32 board, sends one safe test frame
# (0x121 [00] = speed PID disabled), and prints a verdict. It leaves can0 up in
# normal mode, ready for: ros2 launch golfcart_low_level low_level.launch.py
set -u

IF=can0
BITRATE=250000
LISTEN_S=12
OUT=$(mktemp /tmp/can_check_listen.XXXXXX)
OUT_TX=$(mktemp /tmp/can_check_tx.XXXXXX)

state() { ip -details link show "$IF" | sed -n 's/.*can .*state \([A-Z-]*\).*/\1/p'; }

if pgrep -f 'golfcart_low_level/(ros_can_bridge|steering_node)' >/dev/null; then
  echo "!! ros_can_bridge / steering_node are still running."
  echo "   Stop the launch (Ctrl-C) and run this again."
  exit 1
fi
if ! ip link show "$IF" >/dev/null 2>&1; then
  echo "!! $IF not found. Is the USB2CAN adapter plugged in?"
  exit 1
fi

echo "== 1/3  Reset $IF: normal mode, $BITRATE bps, TX queue flushed"
sudo ip link set "$IF" down
if ! sudo ip link set "$IF" up type can bitrate "$BITRATE"; then
  echo "!! could not bring $IF up"
  exit 1
fi
echo "   $IF state: $(state)"

echo
echo "== 2/3  Listening for ${LISTEN_S} s"
stdbuf -oL candump -e -t d "$IF" > "$OUT" 2>&1 &
CD=$!
sleep 0.5
echo "   >>> NOW: switch the STM32 motor/brake board OFF, wait 2 s, then ON again <<<"
echo "   >>> (cart power / KEYA steering driver must be ON too)                   <<<"
for ((i = LISTEN_S; i > 0; i--)); do printf '\r   %2d s left ' "$i"; sleep 1; done
echo
kill "$CD" 2>/dev/null
wait "$CD" 2>/dev/null

n_err=$(grep -c ERRORFRAME "$OUT")
ids=$(grep -v ERRORFRAME "$OUT" | awk -v ifc="$IF" '$2 == ifc {print $3}')
n_all=$(printf '%s\n' "$ids" | grep -c .)
n_stm=$(printf '%s\n' "$ids" | grep -c -E '^(122|123|131|133|134)$')
n_keya=$(printf '%s\n' "$ids" | grep -c -E '^070000[0-9A-F]{2}$')

echo "   frames by ID (count  ID):"
if [ "$n_all" -gt 0 ]; then
  printf '%s\n' "$ids" | grep . | sort | uniq -c | sort -rn | head -15 | sed 's/^/     /'
else
  echo "     (none)"
fi
echo "   error frames: $n_err"
if [ "$n_err" -gt 0 ]; then
  echo "   first error frames:"
  grep -A3 ERRORFRAME "$OUT" | head -12 | sed 's/^/     /'
fi

echo
echo "== 3/3  TX test: one safe frame 0x121 [00] (speed PID disabled)"
stdbuf -oL candump -e -t d "$IF" > "$OUT_TX" 2>&1 &
CD=$!
sleep 0.3
tx0=$(cat "/sys/class/net/$IF/statistics/tx_packets")
cansend "$IF" 121#00
sleep 1
tx1=$(cat "/sys/class/net/$IF/statistics/tx_packets")
kill "$CD" 2>/dev/null
wait "$CD" 2>/dev/null
st=$(state)
n_err_tx=$(grep -c ERRORFRAME "$OUT_TX")
echo "   tx_packets $tx0 -> $tx1, state $st, error frames $n_err_tx"
if [ "$tx1" -gt "$tx0" ] && [ "$st" = "ERROR-ACTIVE" ] && [ "$n_err_tx" -eq 0 ]; then
  tx_ok=1
else
  tx_ok=0
  # The unACKed frame would sit in the adapter retrying forever; flush it so
  # the next launch starts from a clean link.
  sudo ip link set "$IF" down
  sudo ip link set "$IF" up type can bitrate "$BITRATE"
fi

echo
echo "================ RESULT ================"
if [ "$n_stm" -gt 0 ]; then
  echo "  [OK]  STM32 board is talking ($n_stm frames of 0x122/0x131/...)"
else
  echo "  [--]  NO frames from the STM32 board (0x131 / 0x122)"
fi
if [ "$n_keya" -gt 0 ]; then
  echo "  [OK]  KEYA steering heartbeat seen ($n_keya frames)"
else
  echo "  [--]  NO KEYA steering heartbeat (0x07000001)"
fi
if [ "$tx_ok" -eq 1 ]; then
  echo "  [OK]  test frame was ACKed -> another node is on the bus"
else
  echo "  [!!]  test frame was NOT ACKed (state=$st)"
fi
if [ "$n_err" -gt 0 ] || [ "$n_err_tx" -gt 0 ]; then
  echo "  [!!]  CAN error frames on the bus ($n_err while listening, $n_err_tx during TX)"
fi
echo
if [ "$n_stm" -gt 0 ] && [ "$tx_ok" -eq 1 ]; then
  echo "  => Bus is healthy. Launch now, and do NOT bring can0 down again first:"
  echo "     ros2 launch golfcart_low_level low_level.launch.py"
elif [ "$n_all" -eq 0 ] && [ "$n_err" -eq 0 ] && [ "$tx_ok" -eq 0 ]; then
  echo "  => Bus is completely silent: no device is on it."
  echo "     Check power to the STM32 board and the KEYA driver, and that the CAN"
  echo "     cable really connects the USB2CAN adapter to the cart's CAN bus."
elif [ "$n_all" -eq 0 ] && [ "$n_err" -gt 0 ]; then
  echo "  => Something is transmitting, but it can't be decoded."
  echo "     Likely CANH/CANL swapped, missing 120 ohm termination, no common GND,"
  echo "     or a bitrate mismatch (STM32 firmware clock/crystal setting)."
else
  echo "  => Mixed result: send the whole output above to Claude."
fi
echo
echo "  raw captures: $OUT  $OUT_TX"

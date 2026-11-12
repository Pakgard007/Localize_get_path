#!/usr/bin/env bash
set -euo pipefail

CAN_IF="${1:-can0}"
BITRATE="${2:-250000}"

sudo modprobe can || true
sudo modprobe can_raw || true
sudo modprobe can_dev || true
sudo modprobe gs_usb || true

if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
  echo "CAN interface ${CAN_IF} was not found. Check the USB2CAN adapter and driver."
  exit 1
fi

sudo ip link set "${CAN_IF}" down || true
sudo ip link set "${CAN_IF}" up type can bitrate "${BITRATE}" restart-ms 100
sudo ip link set "${CAN_IF}" txqueuelen 1000
ip -details -statistics link show "${CAN_IF}"

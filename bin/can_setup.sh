#!/bin/bash

temp0="can_temp_0"
temp1="can_temp_1"

get_iface() {
  dev_path=$(udevadm info -q path -n /dev/can_usb_$1 2>/dev/null)
  find /sys/class/net -lname "*$dev_path*" -exec basename {} \; 2>/dev/null
}

iface0=$(get_iface 0)
iface1=$(get_iface 1)

if [ -z "$iface0" ] || [ -z "$iface1" ]; then
  echo "Could not identify one or both CAN interfaces"
  exit 1
fi

for iface in "$iface0" "$iface1"; do
  sudo ip link set "$iface" down || true
done

if [ "$iface0" = "can1" ] || [ "$iface0" = "can0" ]; then
  sudo ip link set "$iface0" name "$temp0"
  iface0="$temp0"
fi
if [ "$iface1" = "can0" ] || [ "$iface1" = "can1" ]; then
  sudo ip link set "$iface1" name "$temp1"
  iface1="$temp1"
fi

sudo ip link set "$iface0" name "can0"
sudo ip link set "$iface1" name "can1"

for iface in can0 can1; do
  sudo ip link set "$iface" type can bitrate 1000000
  sudo ip link set dev "$iface" txqueuelen 1000
  sudo ip link set "$iface" up
done


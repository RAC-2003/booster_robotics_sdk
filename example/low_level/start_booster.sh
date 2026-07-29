#!/bin/bash
# Connect the robot to the internet by forwarding it through this laptop's
# connection over SSH.
#
# Run this ON THE LAPTOP (not on the robot). It:
#   1. Sets up IP forwarding + NAT on the laptop (local sudo password
#      requested if needed).
#   2. SSHes into the robot and sets its default route + DNS there (remote
#      sudo password requested if needed).
#
# Requires the robot already reachable at $ROBOT_SSH (i.e. the laptop is
# connected to the robot's uap0 Wi-Fi AP).

set -e

LAPTOP_WAN_IF="enxb8cb29fce6fa"   # laptop's internet-facing interface
LAPTOP_LAN_IF="wlp0s20f3"        # laptop's interface facing the robot AP
ROBOT_SSH="booster@192.168.50.1"
ROBOT_GATEWAY="192.168.50.203"   # laptop's IP as seen from the robot
ROBOT_IF="uap0"

echo "== Laptop: enabling IP forwarding + NAT (sudo password may be requested) =="
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -P FORWARD ACCEPT
sudo iptables -t nat -A POSTROUTING -o "$LAPTOP_WAN_IF" -j MASQUERADE
sudo iptables -A FORWARD -i "$LAPTOP_LAN_IF" -o "$LAPTOP_WAN_IF" -j ACCEPT
sudo iptables -A FORWARD -i "$LAPTOP_WAN_IF" -o "$LAPTOP_LAN_IF" -j ACCEPT

echo "== Robot: setting default route + DNS over SSH (robot sudo password may be requested) =="
ssh -t "$ROBOT_SSH" "
  sudo ip route add default via $ROBOT_GATEWAY dev $ROBOT_IF &&
  echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf &&
  curl -s --max-time 5 https://www.google.com > /dev/null && echo 'Internet OK' || echo 'Internet check failed'
"

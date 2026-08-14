#!/bin/bash
set -euo pipefail

CONNECTION_NAME=${CONNECTION_NAME:-thor-wired-pc}
HOST_IP=${HOST_IP:-192.168.10.2}
PREFIX=${PREFIX:-24}
IFACE=${1:-${IFACE:-}}

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    exec sudo CONNECTION_NAME="$CONNECTION_NAME" HOST_IP="$HOST_IP" PREFIX="$PREFIX" IFACE="$IFACE" "$0" "$@"
fi

if ! command -v nmcli >/dev/null 2>&1; then
    echo "nmcli not found. Install/use NetworkManager on the host first." >&2
    exit 1
fi

pick_ethernet_iface() {
    local connected
    connected=$(nmcli -t -f DEVICE,TYPE,STATE device status | awk -F: '$2 == "ethernet" && $3 == "connected" {print $1}' | head -n 1)
    if [[ -n "$connected" ]]; then
        echo "$connected"
        return
    fi

    nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "ethernet" {print $1}' | head -n 1
}

if [[ -z "$IFACE" ]]; then
    IFACE=$(pick_ethernet_iface)
fi

if [[ -z "$IFACE" ]]; then
    echo "No ethernet interface found. Pass one explicitly, e.g. IFACE=enp1s0 $0" >&2
    exit 1
fi

if ! nmcli -t -f DEVICE device status | grep -Fxq "$IFACE"; then
    echo "NetworkManager does not know interface '$IFACE'." >&2
    nmcli device status >&2
    exit 1
fi

if nmcli -t -f NAME connection show | grep -Fxq "$CONNECTION_NAME"; then
    nmcli connection modify "$CONNECTION_NAME" connection.interface-name "$IFACE"
else
    nmcli connection add type ethernet ifname "$IFACE" con-name "$CONNECTION_NAME"
fi

nmcli connection modify "$CONNECTION_NAME" \
    ipv4.method manual \
    ipv4.addresses "$HOST_IP/$PREFIX" \
    ipv4.gateway "" \
    ipv4.dns "" \
    ipv4.never-default yes \
    ipv6.method disabled \
    connection.autoconnect yes

nmcli connection up "$CONNECTION_NAME"

cat <<EOF
Configured $IFACE with NetworkManager profile '$CONNECTION_NAME':
  Host IP: $HOST_IP/$PREFIX

Set the Windows Ethernet adapter manually to:
  IP address: 192.168.10.1
  Subnet mask: 255.255.255.0
  Gateway/DNS: blank

From Windows, connect to this host at:
  ping $HOST_IP
  rosbridge: ws://$HOST_IP:9093
EOF

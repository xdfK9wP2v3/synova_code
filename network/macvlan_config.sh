#!/bin/bash

subnet=${SUBNET:-12}
echo ">>> subnet：$subnet"

create_macvlan_interface() {
    local IFACE_NAME="$1"
    local IP_ADDR="$2"
    local PARENT_IFACE="enp2s0f1np1"
    local MACVLAN_MODE="bridge"

    sudo ip link del macvlan-shim
    sudo ip link add "$IFACE_NAME" link "$PARENT_IFACE" type macvlan mode "$MACVLAN_MODE"
    sudo ip addr add "$IP_ADDR" dev "$IFACE_NAME"
    sudo ip link set "$IFACE_NAME" up

    sudo ip link set "$IFACE_NAME" mtu 9000
    sudo ip link set docker0 mtu 9000
}

create_macvlan_interface "macvlan-shim" "192.168.${subnet}.2/24"
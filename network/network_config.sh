sudo ethtool -G enp2s0f1np1 tx 8160
sudo ethtool -G enp2s0f1np1 rx 8160
sudo ethtool -g enp2s0f1np1

sudo sysctl -w net.core.wmem_default=100000000
sudo sysctl -w net.core.wmem_max=100000000
sudo sysctl -w net.core.rmem_default=100000000
sudo sysctl -w net.core.rmem_max=100000000

sudo sysctl -w net.core.netdev_max_backlog=6000

sudo ifconfig enp2s0f1np1 mtu 9000 up

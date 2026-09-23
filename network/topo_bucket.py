#!/usr/bin/env python3
# G33
from mininet.net import Mininet
from mininet.node import OVSSwitch
from mininet.cli import CLI
from mininet.link import TCLink
from mininet.log import setLogLevel


def build_topology(bandwidth_mbps):
    net = Mininet(switch=OVSSwitch, link=TCLink)

    h1 = net.addHost('h1', ip='10.0.0.1/24')
    h2 = net.addHost('h2', ip='10.0.0.2/24')
    
    # เพิ่ม failMode='standalone' เพื่อให้ s1 จัดการการส่ง Packet เองโดยไม่ต้องรอ Controller
    s1 = net.addSwitch('s1', failMode='standalone')

    # จำกัด bandwidth ที่ link ตามค่าประจำกลุ่ม
    net.addLink(h1, s1, bw=bandwidth_mbps)
    net.addLink(h2, s1, bw=bandwidth_mbps)

    net.start()
    return net


if __name__ == '__main__':
    setLogLevel('info')
    BANDWIDTH = 5  # bandwidth_mbps G33
    net = build_topology(BANDWIDTH)
    CLI(net)
    net.stop()
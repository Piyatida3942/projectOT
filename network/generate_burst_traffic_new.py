#!/usr/bin/env python3

from scapy.all import IP, UDP, Raw, send
import time
import sys


def send_burst(dst_ip, burst_size, packet_size, interval_ms, num_bursts=10):
    payload = b'X' * packet_size
    for burst_num in range(num_bursts):
        print(f'[Burst {burst_num + 1}] ส่ง {burst_size} packets')
        for _ in range(burst_size):
            pkt = IP(dst=dst_ip) / UDP(dport=5000) / Raw(load=payload)
            send(pkt, verbose=0)
        time.sleep(interval_ms / 1000)




if __name__ == '__main__':
    DST_IP = "10.0.0.2"
    BURST_SIZE = 82       
    PACKET_SIZE = 512     
    INTERVAL_MS = 100     

    send_burst(DST_IP, BURST_SIZE, PACKET_SIZE, INTERVAL_MS)
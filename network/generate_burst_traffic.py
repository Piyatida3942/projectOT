#!/usr/bin/env python3
from scapy.all import IP, UDP, Raw, conf
import time
import sys


def send_burst(dst_ip, burst_size, packet_size, interval_ms, num_bursts=10):
    payload = b'X' * packet_size
    pkt = IP(dst=dst_ip) / UDP(dport=5000) / Raw(load=payload)

    s = conf.L3socket()  # เปิด socket ครั้งเดียว ใช้ส่งซ้ำตลอดทั้งโปรแกรม
    ok = 0
    errors = 0
    t_start = time.time()
    try:
        for burst_num in range(num_bursts):
            print(f'[Burst {burst_num + 1}] ส่ง {burst_size} packets')
            for _ in range(burst_size):
                try:
                    s.send(pkt)
                    ok += 1
                except OSError:
                    errors += 1
            time.sleep(interval_ms / 1000)
    finally:
        s.close()

    elapsed = time.time() - t_start
    total = ok + errors
    print(f'สรุป: เรียก send สำเร็จ {ok}, error {errors}, '
          f'รวม {total} (ควรเท่ากับ {burst_size * num_bursts})')
    print(f'เวลาที่ใช้ทั้งหมด: {elapsed:.3f} วินาที '
          f'(เฉลี่ย {elapsed / total * 1000:.2f} ms/packet)')


if __name__ == '__main__':
    DST_IP = "10.0.0.2"
    BURST_SIZE = 82       
    PACKET_SIZE = 512     
    INTERVAL_MS = 100     

    send_burst(DST_IP, BURST_SIZE, PACKET_SIZE, INTERVAL_MS)
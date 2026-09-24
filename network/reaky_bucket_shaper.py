#!/usr/bin/env python3
# Group: G33 - Leaky Bucket Implementation
import time
import threading
import queue

class LeakyBucket:
    def __init__(self, drain_rate_pps, bucket_capacity):
        self.drain_rate = drain_rate_pps
        self.capacity = bucket_capacity
        self.buffer = queue.Queue(maxsize=bucket_capacity)
        self.dropped = 0
        self.sent = 0
        self._running = True

    def add_packet(self, packet):
        try:
            self.buffer.put_nowait(packet)
        except queue.Full:
            self.dropped += 1
            print(f"[DROP] Buffer เต็ม! {packet} ถูกทิ้ง (Total Dropped: {self.dropped})")

    def start_draining(self, output_callback):
        interval = 1.0 / self.drain_rate
        def drain_loop():
            while self._running:
                try:
                    packet = self.buffer.get(timeout=interval)
                    output_callback(packet)
                    self.sent += 1
                except queue.Empty:
                    pass
                time.sleep(interval)

        threading.Thread(target=drain_loop, daemon=True).start()

    def stop(self):
        self._running = False

if __name__ == '__main__':
    def output(pkt):
        print(f"[SEND {time.time():.4f}] {pkt}")

    # คำนวณค่าสำหรับกลุ่ม G33:
    # drain_rate_pps = (1024 * 1024) / (512 * 8) = 256 packets/sec
    # bucket_capacity = 82 packets
    DRAIN_RATE_PPS = 256
    BUCKET_CAPACITY = 82

    bucket = LeakyBucket(drain_rate_pps=DRAIN_RATE_PPS, bucket_capacity=BUCKET_CAPACITY)
    bucket.start_draining(output)

    print("--- เริ่มจำลอง Bursty Input เข้า Leaky Bucket (100 Packets) ---")
    # ใส่ 100 packets เข้าถังทันทีรวดเดียวโดยไม่ใช้ sleep เพื่อจำลอง Instantaneous Burst
    for i in range(100):
        bucket.add_packet(f"Packet-{i+1}")

    time.sleep(1)  # รอให้ระบาย packet ค้างถังจนหมด
    bucket.stop()

    print(f"\n=== สรุปผลการทำงาน ===")
    print(f"ส่งสำเร็จ: {bucket.sent} packets")
    print(f"ถูกทิ้ง (Dropped): {bucket.dropped} packets")
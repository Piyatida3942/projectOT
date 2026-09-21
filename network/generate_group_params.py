#!/usr/bin/env python3
import hashlib
import random
import sys

def generate_group_params(group_id: str):
    seed = int(hashlib.md5(group_id.encode()).hexdigest(), 16) % (2**32)
    rng = random.Random(seed)

    params = {
        "group_id": group_id,
        "bandwidth_mbps": rng.choice([2, 5, 8, 10, 15]),
        "token_rate_kbit": rng.choice([256, 512, 1024, 2048]),
        "bucket_size_kbit": rng.choice([16, 32, 64, 128]),
        "burst_size_packets": rng.randint(20, 100),
        "burst_interval_ms": rng.choice([50, 100, 200, 500]),
        "packet_size_bytes": rng.choice([512, 1024, 1400]),
        "algorithm": rng.choice(["token_bucket", "leaky_bucket"]),
        "latency_ms": rng.choice([50, 100, 200, 400]),
    }
    return params

if __name__ == "__main__":
    group_id = sys.argv[1] if len(sys.argv) > 1 else "G00"
    p = generate_group_params(group_id)
    print(f"=== Parameter สำหรับกลุ่ม {p['group_id']} ===")
    for k, v in p.items():
        if k != "group_id":
            print(f"  {k}: {v}")
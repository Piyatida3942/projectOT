import numpy as np

# พารามิเตอร์และสุ่มข้อมูลชุดเดิม
n, N, p = 10000, 10, 0.365
np.random.seed(42)
X = np.random.binomial(N, p, n)

# คำนวณหา S_n (Sample Standard Deviation) ด้วย numpy (แก้เป็น np.std)
S_n = np.std(X, ddof=1)

print(f"Sample Standard Deviation (S_n) = {float(S_n):.4f}")
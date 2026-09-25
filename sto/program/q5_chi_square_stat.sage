import numpy as np
from scipy.stats import binom

# 1. พารามิเตอร์ตั้งต้น (p = 0.365)
n, N, p = 10000, 10, 0.365
np.random.seed(42)
X = np.random.binomial(N, p, n)

# 2. คำนวณค่า M_n, p_n และความถี่ (แก้เป็น np.mean)
M_n = np.mean(X)
p_n = M_n / N
H = np.bincount(X, minlength=N + 1)
k_vals = list(range(N + 1))
expected_freq = [n * binom.pmf(k, N, p_n) for k in k_vals]

# 3. คำนวณค่าสถิติทดสอบ Z
Z_terms = [((H[j] - expected_freq[j])**2) / expected_freq[j] for j in range(N + 1)]
Z = sum(Z_terms)

# 4. แสดงผลตารางด้วย SageMath table()
rows_q5 = []
for j in range(N + 1):
    rows_q5.append([j, int(H[j]), round(float(expected_freq[j]), 2), round(float(Z_terms[j]), 4)])

t5 = table(rows_q5, header_row=["Value (j)", "Observed Freq (H_j)", "Expected Freq (n*PMF)", "Chi-Sq Component"])

print(f"Sample Mean (M_n) = {float(M_n):.4f}")
print(f"Estimated p (p_n) = {float(p_n):.5f}")
print(f"Test Statistic (Z) = {float(Z):.4f}\n")
show(t5)
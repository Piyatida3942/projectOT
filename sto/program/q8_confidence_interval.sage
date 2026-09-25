import numpy as np
from scipy.stats import t

# พารามิเตอร์และสุ่มข้อมูลชุดเดิม
n, N, p = 10000, 10, 0.365
np.random.seed(42)
X = np.random.binomial(N, p, n)

# แก้เป็น np.mean และ np.std
M_n = np.mean(X)
S_n = np.std(X, ddof=1)
alpha = 0.05

# คำนวณ y_alpha/2 และช่วงความเชื่อมั่น 95%
yThreshold2 = t.ppf(1 - alpha / 2, df=n - 1)
margin_of_error = (S_n * yThreshold2) / np.sqrt(n)
lowerCI = M_n - margin_of_error
upperCI = M_n + margin_of_error

# แสดงผลตารางด้วย SageMath table()
rows_q8 = [
    ["Sample Mean (M_n)", f"{float(M_n):.4f}"],
    ["Sample Standard Deviation (S_n)", f"{float(S_n):.4f}"],
    ["Critical Value (y_alpha/2)", f"{float(yThreshold2):.4f}"],
    ["Margin of Error (delta)", f"{float(margin_of_error):.4f}"],
    ["95% Lower Bound (lowerCI)", f"{float(lowerCI):.4f}"],
    ["95% Upper Bound (upperCI)", f"{float(upperCI):.4f}"],
    ["95% Confidence Interval", f"[{float(lowerCI):.4f}, {float(upperCI):.4f}]"]
]

t8 = table(rows_q8, header_row=["Parameter / Statistic", "Value"])

print(f"y_alpha/2 = {float(yThreshold2):.4f}")
print(f"95% CI = [{float(lowerCI):.4f}, {float(upperCI):.4f}]\n")
show(t8)
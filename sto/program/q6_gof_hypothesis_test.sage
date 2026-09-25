from scipy.stats import chi2

# ค่าพารามิเตอร์จากข้อ 5
N = 10
m = N + 1    # Bins = 11
r = 1        # พารามิเตอร์ p_n
alpha = 0.05
Z = 4.5927  # ค่า Z จากข้อ 5

# คำนวณ Degrees of Freedom และ Threshold
kDegree = m - 1 - r
zThreshold = chi2.ppf(1 - alpha, kDegree)

# สรุปผลการทดสอบ
conclusion = "A candidate PMF is a good fit" if Z <= zThreshold else "NOT a good fit"

# แสดงผลตารางด้วย SageMath table()
rows_q6 = [
    ["Number of Bins (m)", m],
    ["Degrees of Freedom (k)", kDegree],
    ["Test Statistic (Z)", f"{float(Z):.4f}"],
    ["Critical Threshold (z_alpha)", f"{float(zThreshold):.4f}"],
    ["Test Conclusion", conclusion]
]

t6 = table(rows_q6, header_row=["Parameter / Statistic", "Value"])

print(f"Degrees of Freedom (k) = {kDegree}")
print(f"Threshold (z_alpha) = {float(zThreshold):.4f}\n")
show(t6)
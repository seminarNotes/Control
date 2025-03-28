import numpy as np
import matplotlib.pyplot as plt

# 설정: 자연 진동수
omega_n = 5.0

# 감쇠비 범위
zeta_values = np.arange(-2.0, 2.05, 0.05)

# 극점 저장 리스트
poles_real = []
poles_imag = []

for zeta in zeta_values:
    real_part = -zeta * omega_n
    imag_part = omega_n * np.sqrt(np.abs(1 - zeta**2))  # 허수부
    if zeta <= 1:
        poles_real.append([real_part, real_part])
        poles_imag.append([imag_part, -imag_part])
    else:
        poles_real.append([real_part, real_part])
        poles_imag.append([0, 0])  # 과감쇠는 실근 2개

# 그래프 그리기
plt.figure(figsize=(8, 6))
for i in range(len(zeta_values)):
    z = zeta_values[i]
    r = poles_real[i]
    im = poles_imag[i]
    label = f"ζ={z:.2f}" if i % 5 == 0 else None  # 라벨은 간격 두고 표시
    plt.plot(r, im, 'o', label=label, markersize=4, color=plt.cm.viridis(i / len(zeta_values)))

# 축 및 안내선
plt.axhline(0, color='k', linewidth=0.5)
plt.axvline(0, color='k', linewidth=0.5)
plt.xlabel("Real Axis")
plt.ylabel("Imaginary Axis")
plt.title("Pole Trajectory for 2nd-order System as ζ varies")
plt.grid(True)
plt.legend(loc='upper right', bbox_to_anchor=(1.25, 1))
plt.axis("equal")
plt.tight_layout()
plt.show()

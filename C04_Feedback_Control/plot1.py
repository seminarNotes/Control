import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, step

# 자연 주파수 설정
omega_n = 5.0  # rad/s

# 감쇠비 범위 설정
zeta_values = np.arange(0.5, 1.51, 0.05)

# 시간 축 설정
t = np.linspace(0, 5, 500)

# 그래프 설정
plt.figure(figsize=(10, 6))
colors = plt.cm.viridis(np.linspace(0, 0.5, len(zeta_values)))

for idx, zeta in enumerate(zeta_values):
    # 전달함수: G(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
    num = [omega_n ** 2]
    den = [1, 2 * zeta * omega_n, omega_n ** 2]
    sys = TransferFunction(num, den)
    
    # 단위 입력 응답 계산
    t_out, y_out = step(sys, T=t)
    
    plt.plot(t_out, y_out, color=colors[idx], label=f"ζ = {zeta:.2f}")

# 그래프 스타일
plt.title("Step Response of 2nd-order System for Varying Damping Ratio ζ")
plt.xlabel("Time [s]")
plt.ylabel("Output y(t)")
plt.grid(True)
plt.legend(loc="right", bbox_to_anchor=(1.25, 0.5))
plt.tight_layout()
plt.show()

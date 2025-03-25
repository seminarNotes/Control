import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# 대상 함수: f(x,y)= x^2 + 2xy + 3y^2 + 4x + 5y + 6
def f(theta):
    x, y = theta
    return x**2 + 2*x*y + 3*y**2 + 4*x + 5*y + 6

# Gradient: ∇f(x,y) = [2x+2y+4, 2x+6y+5]
def grad_f(theta):
    x, y = theta
    return np.array([2*x + 2*y + 4, 2*x + 6*y + 5])

# Gradient Descent 파라미터
alpha = 0.05       # 학습률
max_iter = 50      # 최대 반복 횟수
tol = 1e-6         # 종료 조건

# 초기 파라미터 설정 (예: [3, 3])
theta = np.array([3.0, 3.0])
trajectory = [theta.copy()]

# Gradient Descent 수행
for i in range(max_iter):
    g = grad_f(theta)
    if np.linalg.norm(g) < tol:
        break
    theta = theta - alpha * g
    trajectory.append(theta.copy())

trajectory = np.array(trajectory)

# --- 2D 애니메이션 (등고선 플롯 위 경로) ---
x_vals = np.linspace(-4, 4, 400)
y_vals = np.linspace(-4, 4, 400)
X, Y = np.meshgrid(x_vals, y_vals)
Z = f([X, Y])

fig2d, ax2d = plt.subplots()
contours = ax2d.contour(X, Y, Z, levels=30, cmap='viridis')
line2d, = ax2d.plot([], [], 'ro-', lw=2)
ax2d.set_xlabel('x')
ax2d.set_ylabel('y')
ax2d.set_title('Gradient Descent 2D Animation')

def init2d():
    line2d.set_data([], [])
    return line2d,

def update2d(frame):
    line2d.set_data(trajectory[:frame+1, 0], trajectory[:frame+1, 1])
    return line2d,

ani2d = FuncAnimation(fig2d, update2d, frames=len(trajectory),
                      init_func=init2d, blit=True, interval=200, repeat_delay=1000)

# --- 3D 애니메이션 (표면 플롯 위에서 점의 이동) ---
fig3d = plt.figure()
ax3d = fig3d.add_subplot(111, projection='3d')
surf = ax3d.plot_surface(X, Y, Z, cmap='viridis', alpha=0.7, edgecolor='none')
point3d, = ax3d.plot([], [], [], 'ro', markersize=8)
ax3d.set_xlabel('x')
ax3d.set_ylabel('y')
ax3d.set_zlabel('f(x,y)')
ax3d.set_title('Gradient Descent 3D Animation')

def init3d():
    point3d.set_data([], [])
    point3d.set_3d_properties([])
    line3d.set_data([], [])
    line3d.set_3d_properties([])
    return point3d, line3d

def update3d(frame):
    x_pt = trajectory[frame, 0]
    y_pt = trajectory[frame, 1]
    z_pt = f(trajectory[frame])

    # 점 위치 갱신
    point3d.set_data([x_pt], [y_pt])
    point3d.set_3d_properties([z_pt])

    # 이동 경로 라인 갱신 (흔적)
    line3d.set_data(trajectory[:frame+1, 0], trajectory[:frame+1, 1])
    line3d.set_3d_properties([f(p) for p in trajectory[:frame+1]])

    return point3d, line3d

line3d, = ax3d.plot([], [], [], 'ro-', lw=2)  # trajectory 라인 추가

ani3d = FuncAnimation(fig3d, update3d, frames=len(trajectory),
                      init_func=init3d, blit=True, interval=200, repeat_delay=1000)

plt.show()

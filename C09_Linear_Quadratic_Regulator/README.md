# Linear Quadratic Regulator

<p align="right">
최초 작성일 : 2025-01-14 / 마지막 수정일 : 2025-01-15
</p>

## 1. Linear Quadratic Regulator 의미

## 2. Hamilton Jacobi Bellman Equation

최적 제어에서 일반적으로 주어지는 최적화 문제로부터 HJB 방정식(Hamilton Jacobi Bellman Equation)을 유도한다. 시스템 방정식과 상태 변수의 초기값이 아래와 같이 주어지고,

$$
\dot{x}(t) = f(x, u, t), \quad x(t_0) = x_0
$$

비용 함수가 아래와 같이 주어졌다고 가정하자.

$$
J(x, t_{f}) = \int_{t_0}^{t_{f}} l(x(\tau), u(\tau), \tau) \, d\tau + m(x(t_{f}))
$$

여기서, $l(x(\tau), u(\tau), \tau)$은 순간 비용 함수를 나타내고, $x(t_{f})$는 종단 비용 함수를 의미한다. 비용 함수 $J(x, t_{f})$는 상태 $x(t)$는 $t_{f}-t_{0}$ 시간 동안의 총 비용을 의미하고, dynamic programming의 원리에 의해 비용 함수를 아래와 같이 recurrsive relation으로 나타낼 수 있다.

$$
J(x, t) = \min_u \Big[ l(x, u, t) \Delta t + J(x + \dot{x} \Delta t, t + \Delta t) \Big]
$$

이제 $J(x + \dot{x} \Delta t, t + \Delta t)$를 테일러 전개에 의해 아래와 같이 표현된다.

$$
J(x + \dot{x} \Delta t, t + \Delta t) = J(x, t) + \frac{\partial J}{\partial t} \Delta t + \frac{\partial J}{\partial x}^T \dot{x} \Delta t + o(\Delta t)
$$

이제 전개된 식을 다시 recurrsive relation에 대입하면, 

$$
J(x, t) = \min_u \Big[ l(x, u, t) \Delta t + J(x, t) + \frac{\partial J}{\partial t} \Delta t + \frac{\partial J}{\partial x}^T f(x, u, t) \Delta t \Big]
$$

양변에서  $J(x, t)$을 소거하고, $\Delta t$를 나누어 정리하면 HJB 방정식을 유도할 수 있다.

$$
\frac{\partial J}{\partial t} = - \min_u \Big[ l(x, u, t) + \frac{\partial J}{\partial x}^T f(x, u, t) \Big]
$$

이제 HJB 방정식에서 Hamiltonian $H(x, u, J_{x}, t)$은 아래와 같이 정의된다.

$$
H(x, u, J_{x}, t) = l(x, u, t) + \frac{\partial J}{\partial x}^T f(x, u, t)
$$

이제 HJB 방정식을 Hamiltonian으로

$$
\frac{\partial J}{\partial t} = - \min_u H(x, u, J_{x}, t)
$$

## 3. LTV 시스템에서 LQR 문제
### 3-1. Liear Time-Varying System

이제 LTV 시스템에서 LQR 문제를 다루기 위해 위에서 가정한 시스템을 

$$
\dot{x}(t) = A(t)x(t) + B(t)u(t)
$$

라 가정하고, 비용 함수를 

$$
J = \frac{1}{2} x^T(t_f)H x(t_f) + \frac{1}{2} \int_{t_0}^{t_f} \big( x^T(t)Q(t)x(t) + u^T(t)R(t)u(t) \big) dt
$$

라 가정한다. 이 때, $x(t)$은 상태 벡터, $u(t)$은 제어 입력, $A(t)$은 상태 행렬, $B(t)$은 입력 행렬, $Q(t)$은 상태 오차에 대한 가중치 행렬, $R(t)$은 제어 입력에 대한 가중치 행렬, $H$은 최종 상태에 대한 가중치 행렬을 나타낸다.

### 3-2. Optimal Control Input
Hailtonian의 최적 입력을 찾는 것은 성능 지표 J를 최적화하는 데 충분 조건(sufficient condition)으로 작용한다. 따라서, 위에서 정의한 비용 함수에 대한 Hailtonian을 정의하고, 원래의 최적화 문제를 Hamiltonian의 미분을 통해 해결한다.

$$
H(x(t), u(t), \hat{J}_x, t) = \frac{1}{2} x^T(t)Q(t)x(t) + \frac{1}{2} u^T(t)R(t)u(t) + \hat{J}_x^{T}(t) \big( A(t)x(t) + B(t)u(t) \big)
$$

여기에서 $J_x(t)$은 상태 변수에 대한 비용 함수의 편미분($\partial J / \partial x$)을 의미한다. Hamiltonian $H$의 제어 입력 $u$에 대한 1차 미분과 2차 미분은 각각 아래와 같다.

$$
\begin{align*}
& \frac{\partial H}{\partial u} = R(t)u(t) + B^T(t)J_x(t)\\
& \frac{\partial^{2} H}{\partial u^{2}} = R(t) > 0
\end{align*}
$$

따라서, 최적 제어 입력 $\hat{u}(t)$는 다음을 만족한다.

$$
\frac{\partial H}{\partial u} = R(t)\hat{u}(t) + B^T(t)\hat{J}_x(t) = 0 
$$

이로부터 최적 입력 $\hat{u}(t)$는 아래와 같이 계산된다.

$$
\hat{u}(t) = -R^{-1}(t)B^T(t)\hat{J}_x(t)
$$

### 3-3. Riccati Differential Equation

이제 위에서 구한 최적 제어 입력 $\hat{u}(t)$을 Hamiltonian $H$에 대입한다.

$$
\begin{align*}
H &= \frac{1}{2}x^TQx + \frac{1}{2}u^TRu + \hat{J}^{T}_x(Ax + Bu) \\
&= \frac{1}{2}x^TQx - \frac{1}{2}\hat{J}_x^{T}BR^{-1}B^T\hat{J}_x + \hat{J}_x^{T}Ax
\end{align*}
$$

그러면, LQR을 위한 HJB 방정식을 아래와 같이 얻을 수 있다.

$$
\begin{align*}
0 &= \hat{J}_t(x(t), t) + H(x(t), \hat{J}(x(t)), \hat{J}(x(t), t), t) \\
&= \hat{J}_t(x(t), t) + \frac{1}{2}x(t)^TQ(t)x(t) - \frac{1}{2}\hat{J}_x^{T}(x(t), t)B(t)R^{-1}(t)B^T(t)\hat{J}_x(x(t), t) + \hat{J}_x^{T}(x(t), t)A(t)x(t)
\end{align*}
$$

이제 비용 함수 $\hat{J}(x(t), t)$는 상태 변수 $x(t)$의 quadratic form로 가정하면,

$$
J(x(t), t) = \frac{1}{2} x^T(t)P(t)x(t)
$$

여기서, $P(t)$는 시간에 의존하는 대칭 행렬이다. 위 비용 함수를 HJB 방정식에 대입하면

$$
\begin{align*}
0 & = J_t(x(t), t) + H(x(t), \hat{u}(t), J_x, t) \\
& = \frac{1}{2} x^T(t)\dot{P}(t)x(t) + \frac{1}{2} x^T(t)Q(t)x(t) - \frac{1}{2} J_x^T(t)B(t)R^{-1}(t)B^T(t)J_x(t) + x^T(t)P(t)A(t)x(t) \\
& = \frac{1}{2}x^T(t)\dot{P}(t)x(t) + \frac{1}{2}x^T(t)Q(t)x(t) - \frac{1}{2}x^T(t)P(t)B(t)R^{-1}(t)B^T(t)P(t)x(t) + x^T(t)P(t)A(t)x(t) \\
& = \frac{1}{2}x^T(t)\dot{P}(t)x(t) + \frac{1}{2}x^T(t)Q(t)x(t) - \frac{1}{2}x^T(t)P(t)B(t)R^{-1}(t)B^T(t)P(t)x(t) + \frac{1}{2}x^T(t)P(t)A(t)x(t) + \frac{1}{2}x^T(t)A^{T}(t)P(t)x(t)
\end{align*}
$$

을 얻는다. 여기서 마지막 등호는 이차형식 $x^T(t)P(t)A(t)x(t)$는 $A(t)$가 비대칭 행렬일 경우, 상태 벡터 $x(t)$의 방향에 따라 다른 값을 가질 수 있기 때문에 $\frac{1}{2} x^T \big(P(t) A(t) + A^T(t) P(t)\big) x$로 대칭화를 한 결과이다.


위 방정식의 초기 조건은 아래와 같다.

$$
P(t_f) = H
$$

이제 벡터 $x(t)$는 임의의 벡터이기 때문에, Riccati 방정식을 얻는다.

$$
\dot{P}(t) + Q(t) - P(t)B(t)R^{-1}(t)B^T(t)P(t) + P(t)A(t) + A^T(t)P(t) = 0
$$

위 미분방정식을 해결하기 위한 초기 조건은 $P(t_f) = H$이고, 방정식을 통해 $P(t)$를 계산한다. 계산된 $P(t)$를 통해 최적 제어 입력 $\hat{u}(t)$를 계산한다.

$$
\hat{u}(t) = -R^{-1}(t)B^T(t)P(t)x(t)
$$

## 4. LTI 시스템에서 LQR 문제
### 4-1. Linear Time-Invariant System

이제 LTI 시스템에서 LQR 문제를 다루기 위해 시스템 방정식을 

$$
\dot{x}(t) = Ax(t) + Bu(t)
$$

로 가정하고, 비용 함수를 

$$
J = \frac{1}{2} x^T(t_f)Hx(t_f) + \frac{1}{2} \int_{t_0}^{t_f} \big( x^T(t)Qx(t) + u^T(t)Ru(t) \big) dt
$$

로 정의한다. 여기서 $x(t)$는 상태 벡터, $u(t)$는 제어 입력, $A$는 상태 행렬, $B$는 입력 행렬, $Q$는 상태 오차에 대한 가중치 행렬, $R$는 제어 입력에 대한 가중치 행렬, $H$는 최종 상태에 대한 가중치 행렬이다.

### 4-2. Algebraic Riccati Equation

LTI 시스템에서 비용함수 $J$가 $\hat{J}=x^{T}(t)Px(t)$를 만족한다고 가정했을 때, 아래와 같이 HJB을 만족한다.


$$
\frac{\partial J}{\partial t} = - \min_{u(t)} \big[ u(t)^{T}Ru(t) + x^{T}(t)Qx(t) + 2x^{T}(t)P(Ax(t)+Bu(t)) \big]
$$

그러면 Hailtonian은 완전제곱식으로 표현할 수 있다.

$$
\begin{align*}
H & = u(t)^{T}Ru(t) + x^{T}(t)Qx(t) + 2x^{T}(t)P(Ax(t)+Bu(t)) \\
& = u(t)^{T}Ru(t) + x^{T}(t)Qx(t) + x^{T}(t)PBR^{-1}B^{T}Px(t) - x^{T}(t)PBR^{-1}B^{T}Px(t) + 2x^{T}(t)PAx(t) + 2x^{T}(t)PBu(t) \\
& = \big( u(t) + R^{-1}B^{T}Px(t) \big)^{T}R \big( u + R^{-1}B^{T}Px(t) \big) - 
x^{T}(t)\big( Q + PA + A^{T}P - PBR^{-1}B^{T}P \big) x(t)
\end{align*}
$$

이제 Hamiltonian이 최소가 되기 위해선 완전제곱식인 첫번째 항이 $0$이 되어야 한다.

$$
u(t) + R^{-1}B^{T}Px(t)  = 0
$$

이를 통해 최적 제어 입력 $\hat{u}(t)$를 계산할 수 있다.

$$
\hat{u}(t) = - R^{-1}B^{T}Px(t)
$$

그리고, 최적 제어 입력 $\hat{u}(t)$를 결정하기 위해서는 행렬 $P$가 결정되어야 하는데, 이것은 Hamiltonian의 두 번째 항이 $0$이 되어야 한다.

$$
Q + PA + A^{T}P - PBR^{-1}B^{T}P = 0
$$

위 식을 Algebraic Riccati Equation이라고 하고, 위 식을 통해 행렬 $P$을 결정하고, 최종적으로 최적 제어 입력을 결정한다.

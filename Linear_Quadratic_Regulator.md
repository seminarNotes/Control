# Linear Quadratic Regulator

<p align="right">
최초 작성일 : 2025-01-14 / 마지막 수정일 : 2025-01-14
</p>

## 1. Linear Quadratic Regulator 의미

LQR은 선형 시스템에서 특정 성능 지표를 최소화하도록 최적의 제어 입력을 계산하는 방법이다. LQR은 아래와 같은 Linear Time-varying 시스템과 비용 함수로 정의된다.

$$
\dot{x}(t) = A(t)x(t) + B(t)u(t)
$$

여기서, $x(t)$은 상태 벡터, $u(t)$은 제어 입력, $A(t)$은 상태 행렬, $B(t)$은 입력 행렬이다. LQR은 성능 지표(performance measure)인 아래 비용 함수 $J$를 최소화하는 제어 입력 $u$을 구하는 것이 목표이다.

$$
J = \frac{1}{2} x^T(t_f)H x(t_f) + \frac{1}{2} \int_{t_0}^{t_f} \big( x^T(t)Q(t)x(t) + u^T(t)R(t)u(t) \big) dt
$$

여기서, $Q(t)$은 상태 오차에 대한 가중치 행렬, $R(t)$은 제어 입력에 대한 가중치 행렬, $H$: 최종 상태에 대한 가중치 행렬이다.

## 2. 최적 제어 입력 계산

최적화 문제를 해결 하기 위해, Hailtonian을 정의한다. 

$$
H(x(t), u(t), J^*_x, t) = \frac{1}{2} x^T(t)Q(t)x(t) + \frac{1}{2} u^T(t)R(t)u(t) + J_x^{*T}(t) \big( A(t)x(t) + B(t)u(t) \big)
$$

여기에서 $J_x(t)$은 상태 변수에 대한 비용 함수의 편미분, $\partial J / \partial x$,을 의미한다. Hamiltonian은 제어 입력 $u(t)$에 대해 최소화하면, 최적 제어 입력 $\hat{u}(t)$는 다음을 만족한다.

$$
\begin{align*}
& \frac{\partial H}{\partial u} = R(t)u(t) + B^T(t)J_x(t) = 0 \\
& \frac{\partial^{2} H}{\partial u^{2}} = R(t) > 0
\end{align*}
$$

이로부터 최적 입력 $\hat{u}(t)$는 아래와 같이 계산된다.

$$
\hat{u}(t) = -R^{-1}(t)B^T(t)J^*_x(t)
$$


## 3. 리카티 방정식 유도

비용 함수 $J^*(x(t), t)$는 상태 변수 $x(t)$의 quadratic form로 가정할 수 있다.

$$
J(x(t), t) = \frac{1}{2} x^T(t)K(t)x(t)
$$

여기서,  $K(t)$는 시간에 의존하는 대칭 행렬이다. 위 비용 함수를 HJB 방정식에 대입하면

$$
\begin{align*}
0 & = J_t(x(t), t) + H(x(t), \hat{u}(t), J_x, t) \\
& = \frac{1}{2} x^T(t)\dot{K}(t)x(t) + \frac{1}{2} x^T(t)Q(t)x(t) - \frac{1}{2} J_x^T(t)B(t)R^{-1}(t)B^T(t)J_x(t) + x^T(t)K(t)A(t)x(t)
\end{align*}
$$

을 얻고, 초기 조건은 아래와 같다.

$$
K(t_f) = H
$$

위 과정을 통해 Riccati 방정식을 얻는다.

$$
\dot{K}(t) + Q(t) - K(t)B(t)R^{-1}(t)B^T(t)K(t) + K(t)A(t) + A^T(t)K(t) = 0
$$

위 미분방정식을 해결하기 위한 초기 조건은 $K(t_f) = H$이고, 방정식을 통해 $K(t)$를 계산한다. 계산된 $K(t)$를 통해 최적 제어 입력 $\hat{u}(t)$를 계산한다.

$$
\hat{u}(t) = -R^{-1}(t)B^T(t)K(t)x(t)
$$

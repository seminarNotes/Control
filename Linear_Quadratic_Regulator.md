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
\frac{\partial H}{\partial u} = R(t)u(t) + B^T(t)J_x(t) = 0
$$

이로부터 최적 입력 $\hat{u}(t)$는 아래와 같이 계산된다.

$$
\hat{u}(t) = -R^{-1}(t)B^T(t)J^*_x(t)
$$


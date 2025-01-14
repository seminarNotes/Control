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


## 3. 리카티 방정식 유도

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
J(x(t), t) = \frac{1}{2} x^T(t)K(t)x(t)
$$

여기서,  $K(t)$는 시간에 의존하는 대칭 행렬이다. 위 비용 함수를 HJB 방정식에 대입하면

$$
\begin{align*}
0 & = J_t(x(t), t) + H(x(t), \hat{u}(t), J_x, t) \\
& = \frac{1}{2} x^T(t)\dot{K}(t)x(t) + \frac{1}{2} x^T(t)Q(t)x(t) - \frac{1}{2} J_x^T(t)B(t)R^{-1}(t)B^T(t)J_x(t) + x^T(t)K(t)A(t)x(t) \\
& = \frac{1}{2}x^T(t)\dot{K}(t)x(t) + \frac{1}{2}x^T(t)Q(t)x(t) - \frac{1}{2}x^T(t)K(t)B(t)R^{-1}(t)B^T(t)K(t)x(t) + x^T(t)K(t)A(t)x(t) \\
& = \frac{1}{2}x^T(t)\dot{K}(t)x(t) + \frac{1}{2}x^T(t)Q(t)x(t) - \frac{1}{2}x^T(t)K(t)B(t)R^{-1}(t)B^T(t)K(t)x(t) + \frac{1}{2}x^T(t)K(t)A(t)x(t) + \frac{1}{2}x^T(t)A^{T}(t)K(t)x(t)
\end{align*}
$$

을 얻는다. 여기서 마지막 등호는 이차형식 $x^T(t)K(t)A(t)x(t)$는 $A(t)$가 비대칭 행렬일 경우, 상태 벡터 $x(t)$의 방향에 따라 다른 값을 가질 수 있기 때문에 $\frac{1}{2} x^T \big(K(t) A(t) + A^T(t) K(t)\big) x$로 대칭화를 한 결과이다.


위 방정식의 초기 조건은 아래와 같다.

$$
K(t_f) = H
$$

이제 벡터 $x(t)$는 임의의 벡터이기 때문에, Riccati 방정식을 얻는다.

$$
\dot{K}(t) + Q(t) - K(t)B(t)R^{-1}(t)B^T(t)K(t) + K(t)A(t) + A^T(t)K(t) = 0
$$

위 미분방정식을 해결하기 위한 초기 조건은 $K(t_f) = H$이고, 방정식을 통해 $K(t)$를 계산한다. 계산된 $K(t)$를 통해 최적 제어 입력 $\hat{u}(t)$를 계산한다.

$$
\hat{u}(t) = -R^{-1}(t)B^T(t)K(t)x(t)
$$

위 최적 제어 입력을 시스템 상태를 점진적으로 감소시키며, 안정적이 궤도 추적을 보장한다.


## 4. LQR 문제 솔루션

LQR 문제를 해결하기 위한 방법은 아래 3단계로 요약된다.

1. 시스템 상태 방정식을 설정한다.

$$
\dot{x}(t) = A(t)x(t) + B(t)u(t)
$$

2. 성능 지표를 정의한다.

$$
J = \frac{1}{2}x^T(t_f)H x(t_f) + \frac{1}{2}\int_{t_0}^{t_f} \big( x^T(t)Q(t)x(t) + u^T(t)R(t)u(t) \big)dt
$$

3. 리카티 방정식를 통해 행렬 $K(t)$를 계산하고, 최적 제어 입력 $\hat{u}(t) = -R^{-1}(t)B^T(t)K(t)x(t)$ 을 구한다.

$$
\dot{K}(t) + Q(t) - K(t)B(t)R^{-1}(t)B^T(t)K(t) + K(t)A(t) + A^T(t)K(t) = 0
$$

## 5. Matlab 구현

LQR 문제와 관련되어 ... 

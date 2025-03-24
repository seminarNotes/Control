# Mobile Robot
<p align="right">
최초 작성일   2025-03-01 / 마지막 수정일   2025-03-01
</p>

운동 방정식과 동역학 방정식은 제어 이론에서 시스템의 동작을 수학적으로 모델링하는 도구로, 제어기의 설계와 분석의 기초가 된다. 운동 방정식은 기하학적 움직임만을 다루어 저속 주행이나 초기 제어 설계에 활용되며, 구조가 단순하기 때문에 연산을 빠르게 수행할 수 있다는 장점이 있다. 동역학 방정식은 질량, 마찰, 토크 등의 물리적 요소를 포함하여 실제 시스템의 물리적 요소를 정밀하게 표현하기 때문에, 고속 주행이나 비선형 시스템에 대한 정밀 제어 설계에 활용된다. 특히 모델 예측 제어(MPC)나 최적 제에서는 시스템의 미래 상태를 예측하기 위해 정확한 모델이 필요한데, 이때 운동학과 동역학 모델은 상황에 따라 사용된다.

## 1. 상태 방정식

모바일 로봇(이동 로봇)은 바퀴 등으로 이동할 수 있는 로봇을 의미한다. 두 바퀴의 속력을 동일하게 함하면 직진 운동을 하고, 속력을 다르게 하면 회전 운동을 할 수 있다. 이러한 로봇을 제어하기 위해서는 로봇의 상태와 제어 하기 위한 변수와 상태 방정식을 정의한다. 아래 내용은 2개의 바퀴가 장착된 differentail drive 로봇을 모델링하는 Unicycle 모델 기준으로 작성되었다.

### 1-1. 운동 방정식 (Kinematic Equation)

운동 방정식은 위치(position), 자세/방향(orientation)의 변화율을 기술하는 방정식으로, 속도(선속도, 각속도)를 제어입력으로 한다. 이는 질량, 마찰, 토크, 관성 등 물리적 요소를 무시한 기하학적 모델이다.

기호를 도입하여 다시 기술하면, 2차원의 직교 좌표에서 구동 바퀴가 2개인 모바일 로봇은 시간이 $t$인 경우, 로봇의 상태 $q(t)$는 직교 좌표 $(x(t),y(t))$가 되고, 바라 보는 방향을 x축과의 각도 $\theta(t)$를 이용해서 표현할 수 있다. 이러한 로봇은 위에서 언급했듯, 직진 운동과 회전 운동을 통해 제어할 수 있기 때문에, 선속도 $v(t)$와 각속도 $w(t)$를 통해 표현할 수 있다. 따라서, 로봇에 대한 상태 변수 $q(t)$와 제어 입력 변수 $u(t)$는 아래와 같이 정의된다.

$$
q(t) = 
\begin{bmatrix}
x(t), y(t), \theta(t)
\end{bmatrix}^{T}
,\quad
u(t) =
\begin{bmatrix}
v(t), w(t)
\end{bmatrix}^{T}
$$

위치와 속도 관계에 의해 아래와 같은 미분 방정식으로 표현할 수 있다.

$$
\begin{aligned}
\dot{x}(t) & = v(t) \cos \theta(t) \\
\dot{y}(t) & = v(t) \sin \theta(t) \\
\dot{\theta}(t) & = w(t)
\end{aligned}
$$

이제 행렬-벡터 곱으로 표현하면,

$$
\dot{q}(t) =
\begin{bmatrix}
\dot{x}(t) \\
\dot{y}(t) \\
\dot{\theta}(t)
\end{bmatrix}
= \begin{bmatrix}
v(t) \cos\theta(t) \\
v(t) \sin\theta(t) \\
w(t)
\end{bmatrix}
= \begin{bmatrix}
\cos\theta(t) & 0 \\
\sin\theta(t) & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v(t) \\
w(t)
\end{bmatrix}
$$

따라서, 로봇의 상태 방정식(State equation)이 유도된다.

$$
\boxed{
\dot{q}(t) = J(q(t)) u(t)
}
$$

### 1-2. 동역학 방정식 (Dynamic Equation)

동역학 방정식은 로봇의 운동을 유발하는 실제 물리적 힘, 질량, 마찰, 토크 등의 요소를 포함한 모델로, 로봇의 속도 및 가속도까지 상태 변수로 고려하고, 제어 입력은 가속도 $a(t)$ 또는 토크 $\alpha(t)$로 주어진다. 시간 $t$ 에서의 로봇 상태는 위치  $x(t), y(t)$, 방향 각도 $\theta(t)$, 선속도 $v(t)$, 각속도 $w(t)$로 구성되고 상태 벡터 $x(t)$는 다음과 같이 표현된다

$$
x(t) = \begin{bmatrix}
x(t) \\
y(t) \\
\theta(t) \\
v(t) \\
w(t)
\end{bmatrix}, \quad
u(t) = \begin{bmatrix}
a(t) \\
\alpha(t)
\end{bmatrix}
$$

로봇의 동역학 모델은 아래와 같은 미분방정식으로 표현된다

$$
\begin{aligned}
\dot{x}(t) &= v(t) \cos\theta(t) \\
\dot{y}(t) &= v(t) \sin\theta(t) \\
\dot{\theta}(t) &= w(t) \\
\dot{v}(t) &= a(t) \\
\dot{w}(t) &= \alpha(t)
\end{aligned}
$$

위 식을 상태 벡터와 제어 입력의 행렬 곱 형태로 표현하면 다음과 같다

$$
\dot{x}(t) =
\begin{bmatrix}
v(t) \cos\theta(t) \\
v(t) \sin\theta(t) \\
w(t) \\
a(t) \\
\alpha(t)
\end{bmatrix} =
f(x(t)) + B u(t)
$$

여기서,

$$
f(x) =
\begin{bmatrix}
v \cos\theta \\
v \sin\theta \\
w \\
0 \\
0
\end{bmatrix}, \quad
B =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
1 & 0 \\
0 & 1
\end{bmatrix}
$$

따라서 로봇의 동역학 상태 방정식은 다음과 같이 요약된다 

$$
\boxed{
\dot{x}(t) = f(x(t)) + B u(t)
}
$$



## 2. 에러 상태 방정식

로봇이 정해진 경로를 따라가도록 제어기를 설계해야 한다. 이를 위해 먼저 로봇 시스템을 수학적으로 모델링하여 정해진 궤적을 추적할 수 있도록 설계해야 한다. 로봇이 주어진 경로를 따라가도록 제어하는 문제는 리더-팔로워 문제와 수학적으로 동일한 형태를 갖는다. 

리더-팔로워 문제는 여러 로봇이 함께 이동하는 방법 중 하나로, 다중 로봇 시스템에서 한 대의 리더 로봇(LR)이 목표 궤적을 따라가고, 나머지 팔로워 로봇(FR)들이 이를 따라가도록 설계하는 제어 방식이다. 리더 로봇은 동역학 모델을 기반으로 상태를 업데이트하며 이동하지만, 팔로워 로봇은 리더 로봇의 움직임을 참고하여 자신의 위치 오차를 최소화 하도록 제어 알고리즘을 적용한다. 먼저, 리더 로봇의 상태 $q_{L}$와 팔로워 로봇의 상태 $q_{F}$를 각각 아래와 같이 정의한다.

$$
q_{L} =
\begin{bmatrix}
x_{L} \\ 
y_{L} \\ 
\theta_{L}
\end{bmatrix}
,\quad
q_{F} =
\begin{bmatrix}
x_{F} \\ 
y_{F} \\ 
\theta_{F}
\end{bmatrix}
$$

이제 운동 방정식을 기반으로 하여 다양한 오차를 정의할 수 있는데, 이러한 오차는 목적에 따라 다양하게 정의될 수 있으며, 각 모델은 제어기의 직관성, 분석 용이성, 제어 목적에 따라 선택된다.

| 오차 정의 | 기준 좌표계 | 직관성 | 선형화 용이성 | 적용 분야 |
|------------------------|-------------------|------------|------------------------|-----------------------------|
| Global-Frame Error | Global | 낮음 | 높음 | 측위, 지도 기반 제어 |
| Local-Frame Error | Local(FR) | 높음 | 중간 | MPC, 로컬 제어기 |
| Polar Error | Polar | 높음  | 낮음 | Lyapunov 기반 비선형 제어   |

### 2-1. Global-Frame Error (World Frame Error)

단순히 전역 좌표계에서 리더와 팔로워의 상태 차이를 계산하여 에러를 정의하는 방식이다.

$$
e(t) = q_{L}(t) - q_{F}(t)
$$

이 방식은 수학적으로 선형화와 해석이 용이하다는 강점이 있다. 일반적으로, 로봇의 비선형 운동 방정식을 선형화하여 선형 상태 방정식으로 표현할 때, 특히 Kalman 필터, LQR, 선형 MPC 등의 제어기 및 상태 추정기 설계에서는 Global 좌표계에서 오차를 정의한 상태에서 선형화를 수행하는 방식이 가장 널리 사용된다. 그러나, 제어기 입장에서 보면 오차가 로봇의 실제 제어 입력(속도, 회전)에 직접 연결되기 어렵고, 특히 방향 해석이 복잡해질 수 있다는 단점이 존재한다.

리더-팔로워 시스템에서 전역 좌표계 기준 상태는 다음과 같이 정의된다 

$$
q_{L} =
\begin{bmatrix}
x_{L} \\
y_{L} \\
\theta_{L}
\end{bmatrix}, \quad
q_{F} =
\begin{bmatrix}
x_{F} \\
y_{F} \\
\theta_{F}
\end{bmatrix}, \quad
e(t) =
\begin{bmatrix}
x_{L} - x_{F} \\
y_{L} - y_{F} \\
\theta_{L} - \theta_{F}
\end{bmatrix}
$$

이제, 에러 상태 방정식을 얻기 위해 위 식을 시간에 대해 미분하면 다음과 같이 정리된다 

$$
\dot{e}(t) = \dot{q_{L}}(t) - \dot{q_{F}(t)
$$

로봇의 운동 방정식을 각각 대입하면 

$$
\begin{aligned}
\dot{q}_{L}(t) &=
\begin{bmatrix}
v_{L} \cos\theta_{L} \\
v_{L} \sin\theta_{L} \\
w_{L}
\end{bmatrix}, \quad
\dot{q}_{F}(t) =
\begin{bmatrix}
v_{F} \cos\theta_{F} \\
v_{F} \sin\theta_{F} \\
w_{F}
\end{bmatrix} \\
\Rightarrow \quad
\dot{e}(t) &= 
\begin{bmatrix}
v_{L} \cos\theta_{L} - v_{F} \cos\theta_{F} \\
v_{L} \sin\theta_{L} - v_{F} \sin\theta_{F} \\
w_{L} - w_{F}
\end{bmatrix}
\end{aligned}
$$

위 식은 전역 좌표계에서의 위치/자세 오차에 대한 시간 변화율을 의미하며, 선형화를 통해 다음과 같은 상태방정식 형태로 정리된다 

$$
\dot{e}(t) = A(t) e(t) + B(t) \delta u(t)
$$

여기서  $\delta u(t) = u(t) - u_d(t) $는 제어 입력의 오차이며,  $A(t) $,  $B(t) $는 다음과 같이 정의되는 시변 Jacobian 행렬이다 

$$
A(t) =
\begin{bmatrix}
0 & 0 & -v_d \sin\theta_d \\
0 & 0 & v_d \cos\theta_d \\
0 & 0 & 0
\end{bmatrix}, \quad
B(t) =
\begin{bmatrix}
\cos\theta_d & 0 \\
\sin\theta_d & 0 \\
0 & 1
\end{bmatrix}
$$


### 2-2. Local-Frame Error(Follower-Local Error)

전역 좌표계에서 계산된 두 로봇의 상태 차이 $q_{L}-q_{F}$를 Follower 로봇의 좌표계 기준으로 회전 변환을 적용함으로써, 정의한 에러이다.

$$
e(t) = R(\theta_{F})^{T} (q_{L}(t) - q_{F}(t))
$$

FR를 제어하기 위해 상태 오차 $q_{e}$를 최소화 하도록 하기 때문에 아래와 같이 FR을 기준으로 LR에 대한 상태 추종 오차를 정의한다. 

$$
e = 
\begin{bmatrix}
e_{x} \\ 
e_{y} \\ 
e_{\theta}
\end{bmatrix} 
= \begin{bmatrix}
\cos\theta_{F} & \sin\theta_{F} & 0 \\ 
-\sin\theta_{F} & \cos\theta_{F} & 0 \\ 
0 & 0 & 1
\end{bmatrix}
\left( q_{L} - q_{F} \right)
$$

위 수식은 두 로봇의 상태 $q_{L}$, $q_{F}$의 전역 좌표계에서 계산된 오차 상태를 FR의 국소(local) 좌표계로 변환한 것을 의미한다. 이제 위 상태값을 대입하여 식을 정리하면 아래와 같다.

$$
e = 
\begin{bmatrix}
e_{x} \\ 
e_{y} \\ 
e_{\theta}
\end{bmatrix} 
= \begin{bmatrix}
\cos\theta_{F} & \sin\theta_{F} & 0 \\ 
-\sin\theta_{F} & \cos\theta_{F} & 0 \\ 
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
x_{L} - x_{F} \\ 
y_{L} - y_{F} \\ 
\theta_{L} - \theta_{F}
\end{bmatrix} 
= \begin{bmatrix}
\cos\theta_{F}(x_{L} - x_{F}) + \sin\theta_{F}(y_{L} - y_{F}) \\ 
-\sin\theta_{F}(x_{L} - x_{F}) + \cos\theta_{F}(y_{L} - y_{F}) \\ 
\theta_{L} - \theta_{F}
\end{bmatrix}
$$

그러면, 에러 상태 방정식을 얻기 위해 위 식을 미분하면,

$$
\begin{align*}
\dot{e} 
= \begin{bmatrix}
v_{L} \cos e_{\theta} + w_{F} e_{y} - v_{F} \\
v_{L} \sin e_{\theta} - e_{x} w_{F} \\
w_{L} - w_{F}
\end{bmatrix} =
\begin{bmatrix}
w_{F} e_{y} - v_{F} \\ 
-e_{x} w_{F} \\
-w_{F}
\end{bmatrix} +
\begin{bmatrix}
\cos e_{\theta} & 0 \\
\sin e_{\theta} & 0 \\
0 & 1
\end{bmatrix} u
\end{align*}
$$

을 얻는다.


### 2-3. Polar Coordinate Error

Polar Coordinate Error는 로봇과 목표 사이의 상대적인 위치 관계를 극좌표계로 표현한 오차 모델로, 제어기의 직관성을 극대화하기 위해 자주 사용된다. 전역 좌표계에서 계산된 위치 오차를 기반으로, 상대 거리와 방향 관계를 극좌표 상의 세 가지 요소인 거리 오차  $\rho $, 방향 오차  $\alpha $, 자세 오차  $\beta $로 변환하여 정의한다.

목표 상태  $q_{L} = [x_{L}, y_{L}, \theta_{L}]^T $, 로봇 상태  $q_{F} = [x_{F}, y_{F}, \theta_{F}]^T $가 주어졌을 때, 전역 좌표계 기준 위치 오차는 다음과 같다 

$$
\Delta x = x_{L} - x_{F}, \quad \Delta y = y_{L} - y_{F}
$$

이제 이 위치 오차를 극좌표 형태로 변환하여 다음과 같은 오차 변수들을 정의한다 

$$
\begin{aligned}
\rho &= \sqrt{(\Delta x)^2 + (\Delta y)^2} \\
\alpha &= -\theta_{F} + \arctan2(\Delta y, \Delta x) \\
\beta &= -\theta_{L} + \arctan2(\Delta y, \Delta x)
= \theta_{L} - \theta_{F} - \alpha
\end{aligned}
$$

상태 변수 $\rho$는 로봇과 목표 사이의 거리 오차, $\alpha$는 로봇이 현재 회전 해야할 방향, $\beta$는 최종 목표 방향과 각도 차이를 의미한다. 또한, 위 오차 항들을 시간에 따라 미분하여 오차 상태 방정식을 유도하면 다음과 같이 표현할 수 있다 

$$
\begin{aligned}
\dot{\rho} &= -v_{F} \cos\alpha \\
\dot{\alpha} &= \omega_{L} - \omega_{F} - \frac{v_{F}}{\rho} \sin\alpha \\
\dot{\beta} &= -\omega_{L}
\end{aligned}
$$

이러한 극좌표 오차 방정식은 제어기의 안정성을 Lyapunov 함수 기반으로 증명하거나, 목표점 도달을 보장하는 비선형 제어기에 자주 응용된다. 특히, 로봇이 최종 위치뿐만 아니라 도착 방향까지 정밀하게 제어되어야 하는 환경에서 매우 유용하다.

# Extended State Observer (ESO)
<p align="right">
최초 작성일 : 2025-03-25 / 마지막 수정일 : 2025-03-25
</p>

Extended State Observer(ESO)는 시스템의 상태뿐만 아니라, 모델 불확실성과 외란을 포함하는 전체(총) 외란(total disturbance)을 동시에 추정하는 관측기이다. ESO는 Active Disturbance Rejection Control (ADRC)에서 핵심적인 역할을 하며, 시스템의 외란 보상을 통해 보다 견고한 제어를 가능하게 한다.

## 1. ESO의 수학적 백그라운드

ESO의 기본 아이디어는 원래의 시스템 상태에 총 외란 $d$를 추가하여, 확장된 상태 벡터 $z$를 정의하는 것이다. 예를 들어, 다음과 같이 표현되는 시스템이 있다고 하자.

$$
\begin{aligned}
\dot{x} &= f(x, u) + d, \\
y &= h(x),
\end{aligned}
$$

여기서 $d$는 외란과 모델 불확실성을 포함하는 lumped disturbance이다. ESO는 확장된 상태

$$
z = \begin{bmatrix} x \\ d \end{bmatrix}
$$

를 추정하며, 이를 위해 관측기 설계는 다음과 같은 형태로 이루어진다.

$$
\dot{\hat{z}} = F(\hat{z}, u) + L \left(y - \hat{y}\right),
$$

여기서 $\hat{z}$는 확장된 상태의 추정값, $\hat{y}$는 $\hat{x}$를 이용해 계산한 출력, 그리고 $L$은 관측기 이득(gain)이다. 이 구조를 통해 ESO는 시스템의 상태와 외란을 동시에 추정하여, 제어 알고리즘에서 보상할 수 있도록 한다.


## 2. 선형 시스템에 대한 ESO

선형 시스템의 경우, 시스템은 일반적으로 아래와 같이 표현된다.

$$
\begin{aligned}
\dot{x} &= Ax + B\left(u + d\right), \\
y &= Cx.
\end{aligned}
$$

여기서 $d$는 외란으로, 보통 천천히 변하거나 일정한 값으로 가정한다. 이 경우, 상태 $x$와 외란 $d$를 하나의 확장된 상태

$$
z = \begin{bmatrix} x \\ d \end{bmatrix}
$$

로 결합할 수 있다. 확장된 시스템의 동역학은 다음과 같이 표현된다.

$$
\dot{z} = \begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} z + \begin{bmatrix} B \\ 0 \end{bmatrix} u.
$$

선형 ESO는 위 모델에 대해 관측기 구조를 아래와 같이 설계한다.

$$
\dot{\hat{z}} = \begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix}\hat{z} + \begin{bmatrix} B \\ 0 \end{bmatrix} u + L\left(y - C\hat{x}\right),
$$

여기서 $\hat{z} = \begin{bmatrix}\hat{x}\\ \hat{d}\end{bmatrix}$이며, $L$은 적절한 관측기 이득으로, 주로 pole placement 기법 등을 이용하여 추정 오차가 수렴하도록 선택된다.


## 3. 비선형 시스템에 대한 ESO

비선형 시스템의 경우, 상태 및 출력 모델은 비선형 함수로 표현된다. 예를 들어, 다음과 같은 시스템을 고려하자.

$$
\begin{aligned}
\dot{x} &= f(x) + b\left(u + d\right), \\
y &= x,
\end{aligned}
$$

여기서 $d$는 외란 및 모델 불확실성을 포함한다. 비선형 시스템에서도 ESO를 적용하기 위해 확장된 상태

$$
z = \begin{bmatrix} x \\ d \end{bmatrix}
$$

를 정의하고, 비선형 관측기를 아래와 같이 설계할 수 있다.

$$
\begin{aligned}
\dot{\hat{x}} &= f(\hat{x}) + b\left(u + \hat{d}\right) + l_1\left(y - \hat{x}\right), \\
\dot{\hat{d}} &= l_2\left(y - \hat{x}\right),
\end{aligned}
$$

여기서 $l_1$과 $l_2$는 관측기 이득으로, 시스템의 비선형 특성을 고려하여 고게인(high-gain) 설계 또는 슬라이딩 모드 기법 등을 적용하여 선택할 수 있다. 이 구조를 통해 비선형 시스템에서도 상태와 외란을 동시에 추정할 수 있으며, 추정 오차가 충분히 빠르게 수렴하도록 보장한다.

---

Extended State Observer는 시스템 외란에 대한 보상 및 보정이 중요한 제어 문제에서 매우 유용하게 사용되며, 선형/비선형 시스템 모두에 대해 적용할 수 있는 유연한 관측기 설계 방법이다.

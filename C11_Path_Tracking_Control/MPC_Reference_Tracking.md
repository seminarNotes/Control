# MPC Reference Tracking

<p align="right">
최초 작성일 : 2025-01-14 / 마지막 수정일 : 2025-01-14
</p>

## 1. Reference Tracking 정리  

MPC(모델 예측 제어)는 시스템의 출력이 특정 값(set point)을 추적하도록 설계할 수 있다. 시스템이 다음과 같이 주어졌다고 가정하자.

$$
\begin{align*}
x_{k+1} & = Ax_{k} + Bu_{k} \\
y_{k} &= Cx_{k}
\end{align*}
$$

여기서,
- $x_k$ : 시스템 상태 변수
- $u_k$ : 제어 입력
- $y_k$ : 시스템 출력

을 각각 의미한다. MPC의 목표는 시스템 출력 $y_{k}$가 설정된 참조값 $r$로 수렴하도록 제어하는 것이다. 참조값 $r$이 상수일 때, 제어 목표는 아래와 같이 표현된다.

$$
y_{k} \to r \quad \text{as} \quad k \to \infty
$$

이 목표는 정상 상태(Steady-state)에서 출력 $y_{k}$와 참조값 $r$ 간의 차이를 $0$으로 만드는 것을 의미하고, 이 조건을 zero offset이라고 한다. 이 조건을 만족하기 위해서는 입력 변수의 수 $n_u$가 출력 변수의 수 $n_y$ 이상이여야 한다.

$$
n_u \geq n_y
$$

참조값 $r$에 따라 두 가지 상황이 발생한다. 먼저, $r = 0$인 경우, 시간 $k$가 증가함에 따라 출력 $y_k$, 상태 $x_k$, 제어 입력 $u_k$ 모두 0으로 수렴한다. 하지만, $r \neq 0$인 경우 출력 $y_k$는 참조 값 $r$로 수렴하지만, 상태 $x_k$와 제어 입력 $u_k$가 0으로 수렴한다고는 보장할 수 없다. 이 문제를 해결하기 위해 비용 함수에 상태와 제어 입력의 차이를 최소화하는 항(penalize)을 추가하여 설계할 수 있다.

MPC에서 Reference Tracking 문제를 해결하기 위해 다양한 비용 함수를 생각할 수 있다.

## 2. Reference Tracking Formulation 1 

$$
J_N = \sum_{k=0}^{N-1} \left( (x_k - x_{\text{ref}}(v))^T Q (x_k - x_{\text{ref}}(v)) + (u_k - u_{\text{ref}}(v))^T R (u_k - u_{\text{ref}}(v)) \right)
$$

여기서,
- $x_{\text{ref}}$ : 출력 $y = v$에 대응하는 상태 변수
- $u_{\text{ref}}$ : 출력 $y = v$에 대응하는 제어 입력
- $v$ : 참조값

참조값 $v$을 세팅하는 방법은 두 가지가 있다.

1. $v = r$: 참조 값을 $r$로 설정하여 간단히 제어
2. $v$를 적분 제어로 설정
$$
v_{t+1} = v_t + K_{\text{int}}(y_t - r)

$$

이 방법은 상태 변수와 제어 입력의 오차를 최소화하는 비요 함수를 사용하여, 계산 복잡도가 낮아 비교적 간단한 문제를 해결 할 수 있지만, 과도 응답 성능 및 정확도 저하 등의 문제가 발생할 수 있다.

## 3. Reference Tracking Formulation 2  

Reference Tracking 문제를 해결하는 두 번째 접근 방식은 확장된 상태 공간 모델을 이용한다. 앞에서 접근하는 방식과 다르게 출력 오차와 제어 입력 변화량을 최소화하는 비용 함수를 사용한다.

$$
J_N = \sum_{k=0}^{N-1} \left( e_k^T Q_e e_k + \Delta u_k^T R \Delta u_k \right)
$$

여기서,

- $e_k = y_k - r$ : 출력 오차
- $\Delta u_k = u_k - u_{k-1}$ : 제어 입력 변화량
- $Q_e \geq 0$ : 출력 오차에 대한 가중치 행렬 
- $R > 0$ : 제어 입력 변화량에 대한 가중치 행렬




# Kalman Filter
<p align="right">
최초 작성일 : 2025-03-25 / 마지막 수정일 : 2025-03-25
</p>

Kalman 필터는 선형 동적 시스템의 상태 추정을 위한 최적 추정기로, 노이즈가 존재하는 관측값과 모델 예측을 결합하여 최적의 상태 추정을 수행한다. 본 문서에서는 Kalman 필터의 수학적 백그라운드, 선형 시스템에서의 Kalman 필터, 그리고 비선형 시스템에 대한 확장된 Kalman 필터(Extended Kalman Filter, EKF)와 무향 칼만 필터(Unscented Kalman Filter, UKF)를 설명한다.


## 1. Kalman Filter의 수학적 백그라운드

Kalman 필터는 상태공간 모델(state-space model)을 기반으로 하며, 선형 시스템은 다음과 같이 표현된다.

$$
\begin{aligned}
x_{k+1} &= A x_{k} + B u_{k} + w_{k}, \\
y_{k} &= C x_{k} + v_{k},
\end{aligned}
$$

여기서,  
\- \( x_k \in \mathbb{R}^n \) : 상태 벡터,  
\- \( u_k \in \mathbb{R}^m \) : 제어 입력,  
\- \( y_k \in \mathbb{R}^p \) : 측정값,  
\- \( A, B, C \) : 시스템 행렬,  
\- \( w_k \)와 \( v_k \) : 각각 프로세스 노이즈와 측정 노이즈 (보통 \( w_k \sim \mathcal{N}(0, Q) \), \( v_k \sim \mathcal{N}(0, R) \)로 가정).

Kalman 필터는 **예측(Prediction)** 단계와 **갱신(Update)** 단계로 나누어 진행된다.

예측 단계:

$$
\begin{aligned}
\hat{x}_{k|k-1} &= A \hat{x}_{k-1|k-1} + B u_{k-1}, \\
P_{k|k-1} &= A P_{k-1|k-1} A^T + Q,
\end{aligned}
$$

갱신 단계:

$$
\begin{aligned}
K_k &= P_{k|k-1} C^T \left(C P_{k|k-1} C^T + R\right)^{-1}, \\
\hat{x}_{k|k} &= \hat{x}_{k|k-1} + K_k \left(y_k - C \hat{x}_{k|k-1}\right), \\
P_{k|k} &= (I - K_k C) P_{k|k-1}.
\end{aligned}
$$

여기서 \( K_k \)는 칼만 이득(Kalman Gain)으로, 추정 오차 공분산 \(P\)를 최소화하는 역할을 한다.


## 2. Linear System에서의 Kalman Filter

선형 시스템의 경우, Kalman 필터는 위의 모델에 따라 최적 상태 추정을 수행한다.  
상태 및 관측 모델이 선형이므로, 추정 오차는 Gaussian 분포를 따르며, 칼만 필터는 최소 제곱 오차 추정을 보장한다.  
특히, 시스템의 추정 오차 공분산 \(P\)는 Riccati 방정식을 통해 갱신되며, 다음과 같은 칼만 이득 \(K_k\)가 최적의 형태로 계산된다.

$$
K_k = P_{k|k-1} C^T \left(C P_{k|k-1} C^T + R\right)^{-1}.
$$

이러한 방식은 LQG(Linear Quadratic Gaussian) 제어의 기반이 되며, 센서 노이즈와 모델 불확실성이 있는 환경에서 매우 유용하게 사용된다.

## 3. Nonlinear에 대한 Kalman Filter

비선형 시스템의 경우, 상태 및 관측 모델이 다음과 같이 비선형 함수로 표현된다.

$$
\begin{aligned}
x_{k+1} &= f(x_k, u_k) + w_k, \\
y_{k} &= h(x_k) + v_k.
\end{aligned}
$$

직접적인 Kalman 필터 적용이 어려워지므로, 비선형 문제를 해결하기 위한 여러 확장이 제안되었다.

### 3.1 Extended Kalman Filter (EKF)

EKF는 비선형 함수 \(f\)와 \(h\)를 1차 Taylor 전개를 통해 선형화하여 Kalman 필터를 적용한다.  
선형화는 다음과 같이 Jacobian 행렬을 사용한다.

$$
F_k = \left.\frac{\partial f}{\partial x}\right|_{x=\hat{x}_{k-1|k-1}}, \quad H_k = \left.\frac{\partial h}{\partial x}\right|_{x=\hat{x}_{k|k-1}}.
$$

예측 단계:

$$
\begin{aligned}
\hat{x}_{k|k-1} &= f(\hat{x}_{k-1|k-1}, u_{k-1}), \\
P_{k|k-1} &= F_{k-1} P_{k-1|k-1} F_{k-1}^T + Q.
\end{aligned}
$$

갱신 단계:

$$
\begin{aligned}
K_k &= P_{k|k-1} H_k^T \left(H_k P_{k|k-1} H_k^T + R\right)^{-1}, \\
\hat{x}_{k|k} &= \hat{x}_{k|k-1} + K_k \left(y_k - h(\hat{x}_{k|k-1})\right), \\
P_{k|k} &= (I - K_k H_k) P_{k|k-1}.
\end{aligned}
$$

### 3.2 Unscented Kalman Filter (UKF)

UKF는 상태 분포를 직접 전파하기 위해 sigma point 방식을 사용한다.  
비선형 함수 \(f\)와 \(h\)를 직접 선형화하지 않고, 상태의 분포를 대표하는 sigma point들을 선택한 후, 각 sigma point에 대해 비선형 함수를 적용한다.  
이후, 이들의 가중 평균과 공분산을 계산하여 상태 추정을 수행한다.  
UKF는 EKF보다 더 정확하게 비선형 효과를 캡처할 수 있지만, 계산 복잡도가 다소 증가하는 단점이 있다.

---

Kalman 필터와 그 확장은 상태 추정, 노이즈 제거, 예측 모델 개선 등 다양한 분야에서 활용되며, 제어 시스템, 신호 처리, 로봇 공학 등에서 중요한 역할을 수행한다.

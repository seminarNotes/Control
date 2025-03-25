# Observability
<p align="right">
최초 작성일 : 2024-12-11 / 마지막 수정일 : 2025-01-07
</p>

시스템의 관측 가능성(Observability)은 시스템의 출력으로부터 초기 상태를 유일하게 복원할 수 있는지를 평가하는 개념이다. 여기서는 선형 시불변 시스템(LTI)을 기준으로 설명하며, 시스템은 다음과 같이 표현된다.

$$
\begin{aligned}
\dot{x}(t) &= Ax(t) + Bu(t) \\
y(t) &= Cx(t) + Du(t)
\end{aligned}
$$

대부분의 경우 \(D=0\)로 가정하고, 출력 \(y(t)\)와 상태 \(x(t)\) 사이의 관계를 중심으로 관측 가능성을 분석한다.

---

## 1. State Estimation (관측 가능성의 기본 개념)

관측 가능성이란, 주어진 시간 구간 \([0, T_f]\) 동안의 출력 \(y(t)\)를 통해 초기 상태 \(x(0)\)를 유일하게 결정할 수 있는지를 의미한다. 다시 말해, 서로 다른 초기 상태가 동일한 출력 궤적을 만든다면, 그 시스템은 관측 불가능하다고 볼 수 있다.

출력의 닫힌 형식 해(closed-form solution)는 다음과 같이 표현된다.

$$
y(t) = Ce^{At}x(0) + \int_{0}^{t} Ce^{A(t-s)}Bu(s) \, ds
$$

만약 \(u(t)\)가 알려져 있거나 \(u(t)=0\)라고 가정하면, 출력은 \(x(0)\)의 선형 함수가 되며, 이 선형 관계를 이용하여 초기 상태를 복원할 수 있으면 시스템은 관측 가능하다고 한다.


## 2. Observability

관측 가능성을 정량적으로 평가하기 위해, 관측 가능성 행렬(Observability Matrix)을 도입한다. 주어진 시스템 \((A, C)\)에 대해 관측 가능성 행렬은 아래와 같이 정의된다.

$$
O(A,C) = \begin{bmatrix}
C \\
CA \\
CA^2 \\
\vdots \\
CA^{n-1}
\end{bmatrix}
$$

시스템 \((A, C)\)가 관측 가능하다는 것은, 이 행렬의 열공간(image)이 전체 상태 공간 \(\mathbb{R}^{n}\)와 일치함을 의미하며, 즉

$$
\text{시스템 } (A, C) \text{는 관측 가능하다} \iff \text{rank } O(A,C) = n.
$$

또한, 출력-상태 변환

$$
\Psi_{T}: x(0) \mapsto y(t), \quad t\in[0,T]
$$

가 injective하다는 것도 관측 가능함의 동치 조건이다.

## 3. Observability Gramian

관측 가능 Gramian은 관측 가능성을 또 다른 방식으로 정량화한 도구로, 유한 시간 \(T\) 동안의 출력 정보를 통합하여 정의된다.

$$
W_{o}(T) = \int_{0}^{T} e^{A^{T}s} C^{T} C \, e^{As} ds
$$

\(W_{o}(T)\)는 대칭 행렬이며, 시스템이 관측 가능할 경우 양의 정부호(positive definite)가 된다. 관측 가능 Gramian의 이미지(image)는 시간 \(T\)에서 추정 가능한 상태들의 집합과 동일하며, 다음과 같은 동치 조건을 갖는다.

$$
\begin{aligned}
& \text{시스템 } (A, C) \text{는 관측 가능하다} \\
\textit{if and only if} \quad & \Psi_{T} \text{가 injective하다} \\
\textit{if and only if} \quad & \text{rank } O(A,C) = n \\
\textit{if and only if} \quad & W_{o}(T) \text{가 양의 정부호이다.}
\end{aligned}
$$


## 4. Numerical Implementation

관측 가능 Gramian \(W_{o}(T)\)를 이용하면, 측정된 출력 \(y(t)\)를 바탕으로 초기 상태 \(x(0)\)를 복원할 수 있다. 만약 \(u(t)=0\)라고 가정하면, 출력은

$$
y(t) = Ce^{At}x(0)
$$

이고, 최종 시간 \(T_f\)에서의 출력을 이용한 초기 상태 추정 식은 다음과 같다.

$$
x(0) = W_{o}(T_f)^{-1} \int_{0}^{T_f} e^{A^{T}s} C^{T} y(s) ds.
$$

MATLAB 코드로 구현한 예제는 아래와 같다.

```matlab
% 관측 가능 Gramian 계산
Integrand = @(tau) expm(A' * tau) * C' * C * expm(A * tau); 
Wo = @(T) integral(Integrand, 0, T, 'ArrayValued', true);
Wo_final = Wo(T_final);

% 초기 상태 추정을 위한 적분값 계산
% y_measured는 시간에 따른 출력 값들이 저장된 함수 혹은 배열이다.
% 예를 들어, y_measured(tau)는 tau에서의 측정 출력.
I_y = integral(@(tau) expm(A' * tau) * C' * y_measured(tau), 0, T_final, 'ArrayValued', true);

% 초기 상태 추정
x0_est = inv(Wo_final) * I_y;

disp('x0_estimated = ');
disp(x0_est);
```

위 코드에서 상태 행렬 $A$, 출력 행렬 $C$, 척정된 출력 $y_me


## 5. Minimum Estimation Ellipsoid

관측 가능 Gramian은 측정 잡음이나 불확실성에 의해 발생하는 초기 상태 추정 오차의 한계를 정량화하는 데에도 활용된다. 만약 측정 잡음의 에너지가 \(1\) 이하로 제한된다면, 출력 \(y(t)\)로부터 추정 가능한 초기 상태들의 집합은 다음과 같은 타원체(Minimum Estimation Ellipsoid)로 표현된다.

$$
\text{Ellipsoid} = \{ x \in \mathbb{R}^{n} \mid x^{T}W_{o}(T)^{-1}x \leq 1 \}
$$

이 타원체의 모양은 \(W_{o}(T)\)의 eigenvalue와 eigenvector에 의해 결정되며, 각 축의 길이는 \(\sqrt{\lambda_i(W_{o}(T))}\)에 비례한다. 따라서, 관측 가능 Gramian을 통해 추정 오차의 한계를 분석하고, 효과적인 상태 추정 알고리즘(예: Luenberger Observer, Kalman Filter) 설계에 활용할 수 있다.

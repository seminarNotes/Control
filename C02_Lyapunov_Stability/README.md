# Lyapunov Stability
<p align="right">
최초 작성일 : 2024-12-11 / 마지막 수정일 : 2025-01-07
</p>

Lyapunov 안정성은 동적 시스템의 안정성을 분석하는 가장 강력한 도구 중 하나이다. 본 문서에서는 Lyapunov 안정성의 수학적 배경과 그에 따른 안정성 분석 방법을 수식을 중심으로 상세히 설명한다.



## 1. 기본 개념

동적 시스템
$$
\dot{x}(t) = f(x(t))
$$
에서 $x(t) \in \mathbb{R}^n$은 상태 변수이고, $f: \mathbb{R}^n \to \mathbb{R}^n$는 시스템의 동역학을 나타내는 연속 함수이다. 평형점 $x = 0$의 안정성을 분석할 때, Lyapunov 함수 $V(x): \mathbb{R}^n \to \mathbb{R}$를 사용하여 시스템의 에너지나 “잠재적 함수”로서의 역할을 평가한다.



## 2. Lyapunov 함수

함수 $V(x)$는 다음 조건을 만족하면 평형점 $x = 0$의 안정성 분석에 유용하다.

1. 양의 정함수 (Positive Definiteness):  
   $$V(0) = 0 \quad \text{and} \quad V(x) > 0 \quad \forall x \neq 0.$$
2. 시간 미분의 부정성 (Negative (Semi-)Definiteness):  
   시스템의 동역학에 따라 $V(x)$의 시간 미분은
   $$\dot{V}(x) = \frac{\partial V}{\partial x} f(x) \leq 0 \quad \text{(또는 } < 0 \text{ for } x \neq 0\text{)}.$$

즉, $V(x)$가 아래 조건을 만족하면 Lyapunov 함수라고 한다.
$$
\begin{aligned}
V(0) &= 0, \\
V(x) &> 0 \quad \forall x \neq 0, \\
\dot{V}(x) &= \frac{\partial V}{\partial x} f(x) \leq 0 \quad (\text{or } < 0 \text{ if } x \neq 0).
\end{aligned}
$$



## 3. Lyapunov의 직접법 (Direct Method)

Lyapunov의 직접법을 통해 평형점 $x = 0$의 안정성을 다음과 같이 판별할 수 있다.

- 안정성 (Stability):  
  만약 $V(x)$가 양의 정함수이고, $\dot{V}(x) \leq 0$라면, 평형점 $x = 0$은 안정적이다.

- 점근적 안정성 (Asymptotic Stability):  
  만약 $V(x)$가 양의 정함수이고, $\dot{V}(x) < 0$ for $x \neq 0$라면, 평형점 $x = 0$은 점근적으로 안정적이다. 즉,
  $$\lim_{t\to\infty} x(t) = 0.$$

- 지수 안정성 (Exponential Stability):  
  추가적으로, $V(x)$가 아래와 같은 경계 조건을 만족하면, 평형점은 지수 안정적이다.
  $$
  c_1 \|x\|^2 \leq V(x) \leq c_2 \|x\|^2, \quad \dot{V}(x) \leq -c_3 \|x\|^2,
  $$
  for some constants $c_1, c_2, c_3 > 0$. 이 경우,
  $$
  \|x(t)\| \leq \sqrt{\frac{c_2}{c_1}}\, \|x(0)\|\, e^{-\frac{c_3}{2c_2} t}.
  $$



## 4. Quadratic Lyapunov Functions for Linear Systems

선형 시스템
$$
\dot{x}(t) = Ax(t)
$$
에 대해 일반적인 Lyapunov 함수는 quadratic 형태로 주어진다.
$$
V(x) = x^T P x,
$$
여기서 $P = P^T > 0$는 양의 정부호 행렬이다. 이때, $V(x)$의 시간 미분은
$$
\dot{V}(x) = \frac{\partial V}{\partial x}\dot{x} = x^T (A^T P + P A) x.
$$
따라서, 만약 $A^T P + P A < 0$ (음의 정부호)인 $P > 0$가 존재하면, 평형점 $x = 0$은 점근적으로 안정적이다. 이 조건은 Lyapunov 방정식이나 LMI(Linear Matrix Inequality)를 통해 확인할 수 있으며, 선형 시스템의 Hurwitz 조건과 동치이다.


예를 들어, 주어진 선형 시스템에 대해 임의의 양의 정부호 행렬 $Q > 0$를 선택하고, Lyapunov 방정식
$$
A^T P + P A = -Q
$$
의 해 $P$를 구할 수 있다면, 이 시스템은 점근적으로 안정적이다. 이와 같이 Lyapunov 함수는 시스템의 안정성 분석뿐만 아니라, 제어 설계 및 최적 제어 문제에서도 중요한 역할을 수행한다.

Lyapunov 안정성 이론은 선형 및 비선형 시스템 모두에 적용 가능하며, 시스템이 외란이나 불확실성에 직면했을 때도 안정성을 보장할 수 있는 견고한 분석 도구로 널리 활용된다.

# Lyapunov Stability
<p align="right">
최초 작성일 : 2025-03-25 / 마지막 수정일 : 2025-03-25
</p>

Lyapunov 안정성은 동적 시스템의 안정성을 분석하는 가장 강력한 도구 중 하나이다. Lyapunov 함수의 강력한 점은 선형 시스템의 경우, Lyapunov 함수의 존재성과 시스템의 안정성을 동치 관계를 가지며, 비선형 시스템의 경우, Lyapunov 함수의 존재성만으로 대응하는 시스템의 안정성을 보장할 수 있다는 점이다. 아래에서는 Lyapunov 함수를 소개하고, 함수를 통해 비선형 시스템의 안정성을 설명한다.

아래 글에서는 분석의 단순화를 위해, 2가지를 가정한다. 

1. Equilibrium poin은 원점 $x = 0$이다.
2. 제어 입력이 없는 자율 시스템(Autonomous System)이다.

$$
\dot{x} = f(x)
$$

첫 번째 가정은 일반성을 잃지 않는다. 임의의 평형점 $x_e$가 존재하는 경우, 

$$
x' = x - x_e
$$

을 통해 평형점을 원점으로 이동시켜 해석할 수 있기 때문이다. 따라서 각 평형점에 대해 국소적으로 분석한다면, 이 가정은 무리가 없다. 하지만, 두번째 가정은, 일부 일반성을 상실한다. Real world에서의 많은 시스템들은 제어 입력 $u(t)$, 외란 $d(t)$ 혹은 시간 종속 변수/요소가 포함된 비선형 시스템이다. 이러한 시스템에 대해 단순한 자율 시스템 기반의 Lyapunov 안정성 조건은 충분하지 않으며, Stability(ISS), Control Lyapunov Function(CLF), LaSalle's Invariance Principle, Barbalat's Lemma 등의 확장된 개념이 요구된다.

## 1. Lyapunov Stability

동적 비선형 시스템은 아래와 같이 정의하자.

$$
\dot{x}(t) = f(x(t))
$$

여기서, $x(t) \in R^n$은 상태 변수이고, $f: R^n \to R^n$은 연속 함수이다. 특히, Equilibrium point $x = 0$의 안정성(stability in the sense of Lyapunov)이라는 것은 임의의 $\epsilon > 0$에 대해, 대응하는 $\delta = \delta(\epsilon) > 0$이 존재해서 다음을 만족함을 의미한다.

$$
||x(0)|| < \delta \quad \text{implies} \quad \|x(t)\| < \varepsilon \quad \quad \text{for} \quad  \forall t \ge 0
$$

Equilibrium point $x = 0$ 근방에서 충분히 가까운 초기 조건에서 출발한 상태 변수 $x(t)$는 항상 원점의 열린 근방(open ball)에 포함된다.

## 2. Lyapunov Function

스칼라 함수 $V(x) \in C^{1}$가 Lyapunov 함수인 것은 아래와 같은 조건을 만족하는 경우를 의미한다.

1. Positive Definiteness :  
   $$V(0) = 0 \quad \text{and} \quad V(x) > 0 \quad \forall x \neq 0$$
2. Negative Semi-Definiteness :  
   $$\dot{V}(x) = \frac{\partial V}{\partial x} f(x) \leq 0$$



## 3. 안정성 판별

$$
\boxed{\text{Existence of Lyapunov Function } V(x)} \Rightarrow \boxed{\text{Lyapunov Stability}}
$$

**(증명)** 연속적으로 미분 가능한 스칼라 함수 $V(x)$가 존재해서, 위 조건을 만족한다고 가정하자. 그러면, $V(x)$의 연속성과 Positive Definiteness에 의해, 원점 근처에서 함수 $V(x)$는 유계(bounded)이기 때문에 임의의 $\epsilon > 0$에 대해 $||x|| < \epsilon $를 만족하는 모든 $x$에 대해

$$
V(x) \leq \alpha 
$$

를 만족하는 양수 $\alpha > 0$가 존재한다. 그러면, 위 부등식을 만족하는 상태 변수들의 집합을 $D_{a}$라고 하자.

$$
D_\alpha := \{x \in R^n \mid V(x) \leq \alpha \}
$$

그러면, 집합 $D_{a}$은 Compact하다. 또, 함수 $V(x)$는 시간 변수 $t$에 대해 비증가 함수이기 때문에 아래 부등식을 만족한다.

$$
V(x(t)) \leq V(x(0))
$$
  

이제 초기 조건 $x(0)$에 대해 $V(x(0)) \leq \alpha$를 만족하도록 $\delta > 0$를 작게 선택할 수 있다.

$$
\|x(0)\| < \delta \quad \Rightarrow \quad x(0) \in D_\alpha \quad \Rightarrow \quad V(x(0)) \leq \alpha
$$

그런데, $\dot{V}(x) \leq 0$이므로 $V(x(t))$는 비증가 함수이며, 아래를 만족한다.

$$
V(x(t)) \leq V(x(0)) < \alpha, \quad \forall t \geq 0
$$

따라서, $x(t) \in D_\alpha$를 만족하는 상태 변수 $x(t)$는 $||x(t)|| < \epsilon $을 성립하기 때문에


$$
\forall \varepsilon > 0,\ \exists \delta > 0:\ \|x(0)\| < \delta \Rightarrow \|x(t)\| < \varepsilon,\ \forall t \geq 0
$$

를 만족하며, 이는 시스템이 Lyapunov 안정성의 정의와 일치한다.

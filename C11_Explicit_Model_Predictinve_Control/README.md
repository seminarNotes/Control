# Explicit MPC

<p align="right">
최초 작성일 : 2025-01-16 / 마지막 수정일 : 2025-01-17
</p>

## 1. Explicit MPC 개요

MPC(Model Predictive Control)는 시스템의 상태를 기반으로 다양한 제약 조건을 고려하여 최적의 제어 입력을 계산할 수 있는 제어 방법이다. 특히 Linear Quadratic MPC는 목적 함수와 제약 조건을 선형 및 이차 형태로 설정하여 제어 문제를 수학적으로 명확하게 정의할 수 있다. 하지만, MPC는 매 샘플링 주기마다 최적화를 수행해야 하는 온라인 최적화 방식으로 동작하기 때문에 계산 복잡성이 높고, 실시간으로 제어 입력을 계산해야 하는 빠른 제어 시스템에서는 문제가 될 수 있다. 

이 문제를 해결하기 위해 MPC를 matrix-form으로 변환하여 Quadratic Programming(QP) 문제로 표현하는 방법이 제안되었다. QP 문제는 목적 함수와 제약 조건이 이차 함수와 선형식으로 구성된 최적화 문제로, 다양한 표준화된 알고리즘을 통해 효율적으로 해결할 수 있다. 이러한 변환은 계산 부담을 줄이고, 최적화 문제를 명확한 수학적 형식으로 재정의하여 실시간 제어의 구현 가능성을 높일 수 있다. 하지만, QP로 변환한 MPC 역시 온라인 최적화를 기반으로 하기 때문에 여전히 반복적인 계산의 부담이 존재하며, 고속 동작이 요구되는 상황에서는 완벽한 해결 방법을 제시하진 못했다. 그래서, 고속을 요구하지 않는 반복 작업에 대해서는 QP문제의 MPC를 사용하는 것이 적절할 수 있으나, 고속을 요구하는 문제에서는 다른 해결 방법을 고민해야 한다.

![MPC](https://github.com/seminarNotes/Control/blob/main/Explicit_MPC/Explicit_MPC_MPC.jpg)


높은 속도에 대한 한계를 극복하기 위해 제안된 방법이 바로 Explicit MPC이다. Explicit MPC는 기존 MPC의 온라인 최적화 방식과 다르게, 모든 상태 변수에 대한 최적의 제어 입력을 사전에 오프라인에서 계산한다. 이 과정에서 상태 공간을 분할(partition)하여 각 영역별로 최적화 문제를 해결하고, 그 결과를 룩업 테이블(lookup table)의 형태로 저장한다. 이를 통해 Explicit MPC는 실시간 실행 시 최적화 계산이 필요 없으며, 단순히 현재 상태에 해당하는 영역을 확인하고 미리 계산된 제어 입력을 조회하여 사용할 수 있다.

이 방식은 특히 임베디드 시스템이나 계산 자원이 제한된 환경에서 매우 효과적이다. Explicit MPC는 실시간 최적화를 수행하는 대신, 오프라인 계산으로 최적화의 대부분을 완료하기 때문에 온라인 계산 부담을 크게 줄이고, 고정된 실행 시간을 보장한다. 결과적으로, 이는 빠른 제어가 요구되는 시스템에서 안정성과 효율성을 동시에 제공할 수 있다.

![Explicit MPC](https://github.com/seminarNotes/Control/blob/main/Explicit_MPC/Explicit_MPC_EMPC.jpg)


## 2. Backgrounds

### 2.1 Parametric Programming

기존의 최적화 문제는 아래와 같이 써진다.

$$
J(\hat{u}) = \min_u J(u) \quad \text{subject to} \quad u \in U
$$

여기에서 $u$는 제어 입력 변수이며, 최적화 문제에서는 결정 변수이다. 그러나, 위 문제는 고정된 상태 변수 $x$가 주어졌을 때, 해당하는 상황에서의 최적화 문제를 고려하고 있기 때문에, 상태 변수 $x$가 고정되지 않고, 변화할 수 있을 때, 최적의 제어 입력 변수를 구하는 parametric programming 문제를 해결 해야 한다.

$$
J(\hat{u}, x) = \min_u J(u, x) \quad \text{subject to} \quad u \in U
$$

여기서, $x$는 최적화 문제의 매개변수이며, $x$의 값에 따라 최적화 문제가 변하며, 따라서, 최적의 제어 입력도 $x$에 의존하게 된다.

$$
\hat{u}(x) = \arg \min_u J(u, x)
$$

요약하면, 위처럼 최적화 문제에서 특정 매개변수의 변화에 따라 최적화 문제의 해와 목적 함수가 변하는 것을 다루는 문제를 parametic progamming라고 한다.

### 2.2 Multi-parametric Quadratic Programming

이제 여러개의 파라미터에 의해 변화하는 최적화 문제를 통해 Explicit-MPC를 수학적으로 기술해보자. 여러개의 파라미터의 부등식은 최종적으로 다각형 모양의 임계 영역(critical region)을 만든다.

먼저, 다루고자 하는 문제를 QP에서 유도한 아래 최적화 문제이다.

$$
\begin{align*}
\min_U J_N = \frac{1}{2} U^T H U + q^T U \quad \text{subject to} \quad G U \leq W + T x_0
\end{align*}
$$

여기에서 parametric는 $x_{0}$이고, single value가 아니라 벡터값으로 구성되어 있는 multi-value이다. 또한, 행렬 $H$는 stricktly convex로 가정하여, global optimal solution이 유일하다. 우선 위 문제를 해결하기 위해 적어도 하나의 feasible solution이 있다고 가정한다.

### 2.3 Karush-Kuhn-Tucker Conditions

일반적인 최적화 문제에서 (제약 조건이 정규성 조건을 만족하는 경우) KKT 조건은 필요조건으로 작용한다. 즉, 최적해라면 KKT 조건을 만족하지만, KKT 조건을 만족하는 모든 해가 최적해가 되진 않는다. 대신, KKT 조건을 만족하지 않으면 최적해가 아니라는 사실을 알 수 있기 때문에 최적해의 후보를 판단하기에 강력한 도구로 사용할 수 있다.

그러나, 위에서 정의한 Quadratic Programming에서 $H \geq 0$을 만족하고, 제약 조건이 convex 영역에서 정의된 경우. KKT 조건은 필요충분조건으로 작용한다. 즉, KKT 조건을 통해 최적화 문제의 최적해를 찾을 수 있다.

이제 KKT 조건을 소개하고, 주어진 QP문제에 어떻게 적용할 수 있는지에 대해 생각해보자. 위에서 정의한 QP문제에 대응하는 Lagrangian 함수를 정의한다.

$$
L(U, \lambda) = \frac{1}{2} U^T H U + q^T U + \lambda^T (GU - W - T x_0)
$$

여기서, $\lambda$는 Lagrange 승수 벡터이고, $GU - W - T x_0$는 제약 조건의 residual을 의미한다.

한편, KKT 조건은 최적해 $U$와 승수 $\lambda$가 존재하기 위해서 아래 4가지 조건을 만족해야 한다는 것을 의미한다.

1. Stationarity \
Stationarity 조건은 목적 함수와 제약 조건의 기울기가 상쇄되어야 하는 것을 의미하며, Lagrangian 함수의 $U$에 대한 미분($\nabla_U L$)이 $0$이 되야 한다.

$$
H U + q + G^T \lambda = 0
$$

2. Prime feasibility \
최적해 $\hat{U}$는 항상 문제에서 주어진 원래의 제약 조건을 만족해야 한다.

$$
GU \leq W + T x_0
$$

3. Dual feasibility \
Lagrange 승수 $\lambda$는 항상 $0$ 이상이여야 한다.

$$
\lambda \geq 0
$$

4. Complementary slackness \
활성화된 제약 조건만 최적화 문제에서 영향이 미친다는 것을 의미한다.

$$
\lambda^T (GU - W - T x_0) = 0
$$

## 3. Solution to Explicit MPC

상태 변수의 초기값 $x_{0}^{0}$, 즉, 시스템이 현재 놓여져 있는 상태를 나타내는 변수가 주어져 있다고 가정하자. 그러면, qaudratic programming 문제를 수치적으로 풀어 최적의 제어 입력 $\hat{U}(x_{0}^{0})$을 계산한다.

초기값 $x_{0}^{0}$과 대응하는 최적해 $\hat{U}(x_{0}^{0})$에 대해 Prime feasible 조건에 의해 항상 아래 부등식을 만족한다.

$$
G\hat{U}(x_{0}^{0}) \leq W + T x_{0}^{0}
$$

그래서 등식(equality)와 부등식(ineuqality)를 만족한다는 기준으로 각각 특정 제약 조건이 활성 상태(active)하다와 비활성 상태(inactive)하다고 분류한다.

1. 활성 제약조건(Active Constraints) \
특정 제약 조건이 활성 상태에 있다는 것은 해당 제약 조건이 최적해에서 경계에 도달했다는 것을 의미한다.

$$
\tilde{G}\hat{U}(x_{0}^{0}) = \tilde{W} + \tilde{T} x_{0}^{0}
$$

2. 비활성 제약조건(Inactive Constraints) \
특정 제약 조건이 비활성 상태에 있다는 것은, 해당 제약 조건이 최적해 내부에서 만족된다는 것을 의미한다.

$$
\bar{G}\hat{U}(x_{0}^{0}) < \bar{W} + \bar{T} x_{0}^{0}
$$

QP는 제약 조건이 있는 최적화 문제이기 때문에 모든 제약 조건을 동시에 고려하는 것은 계산면에서 비효율적이다. 따라서, 활성 제약 조건(경계에 있는 조건)만 고려하고, 비활성 제약조건은 무시하는 방식으로 문제를 단순화한다. 따라서, 위 과정은 문제를 단순화하기 위에 활성 제약조건과 비활성 제약조건을 구분하는 단계이다.

이제 $x_{0}^{0}$ 근방의 $x_{0}$를 가정하자. 그러면 Stationity 조건은 아래와 같이 주어며,

$$
H U + q + G^T \lambda = 0
$$

따라서, 최적 제어 입력 $\hat{U}$는 다음과 같이 계산된다.

$$
\hat{U} = H^{-1}[-q - G^T \hat{\lambda}]
$$

$G$의 경우, 전체 제약조건을 만족하는 기호를 의미한다. 그런데, 비활성화 제약 조건의 라그랑지 승수 $\lambda$는 항상 $0$이기 때문에 실제 계산에서는 활성 제약 조건만 영향을 미친다. 따라서, complimentary slackness에 의하면, 아래 등식을 만족한다.

$$
G^T \hat{\lambda} = \tilde{G}^T \hat{\tilde{\lambda}} \quad (\bar{\lambda} = 0)
$$

그래서, 최적 제어 입력 $\hat{U}$는 다음과 같이 다시 쓸 수 있다.

$$
\hat{U} = H^{-1}[-q - \tilde{G}^T \hat{\tilde{\lambda}}]
$$

이제 위 최적 제어 입력을 활성 제약 조건 수식에 대입하고,

$$
\tilde{G}H^{-1}[-q - \tilde{G}^T \hat{\tilde{\lambda}}] = \tilde{W} + \tilde{T} x_{0}
$$

위 식을 $\hat{\tilde{\lambda}}$에 대해 정리해서

$$
\hat{\tilde{\lambda}} = - \left(\tilde{G}H^{-1}\tilde{G}^T\right)^{-1} \left(\tilde{W} + \tilde{T}x_{0} + \tilde{G}H^{-1}q\right)
$$

최종적으로, 최적 제어 입력 $\hat{U}(x_{0})$은 아래와 같다.


$$
\hat{U}(x_{0}) = H^{-1} \left[ -q + \tilde{G}^T \left( \tilde{G}H^{-1}\tilde{G}^T \right)^{-1} \left( \tilde{W} + \tilde{T}x_0 + \tilde{G}H^{-1}q \right) \right]
$$

마지막 식은 $x0$가 선형적으로 나타난다는 점에서, 초기 상태값 $x_{0}^{0}$ 근처에서 최적해 $\hat{U}(x_{0})$는 상태 $x_{0}$의 Affine 함수로 표현된다. 다시 말해, 위 결론은 아래와 같은 형태를 갖는다.

$$
\hat{U}(x_{0}) = K x_{0} + L
$$

여기서, $K$는 상태 $x_{0}$에 대한 선형적인 상호작용을 나타내는 이득 행렬이고, $L$은 상수항을 나타낸다. 기존 MPC 문제와 동일하게 위와 같은 방법으로 최적 제어 입력 $\hat{U}(x_{0})$에서 첫 번째 성분으로 MPC 제어 입력이 결정된다.

$$
u_{MPC}(x_0) = \begin{bmatrix} I_{n_u \times n_u} & 0 & \cdots & 0 \end{bmatrix} \hat{U}(x_0)
$$

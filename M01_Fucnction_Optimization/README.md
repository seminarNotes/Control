# 함수 최적화(Function Optimization)
<p align="right">
최초 작성일   2025-03-25 / 마지막 수정일   2025-03-25
</p>

최적화 하고자 하는 문제는 아래와 같이 일반화된다.
$$
\min_{\theta \in \mathbb{R}^{n}} J(\theta)
$$

여기서, $\theta$는 최적화 변수 혹은 모델의 파라미터 변수이고, $J(\theta)$는 목적 함수(cost / loss function)이다.

## 1. Gradient Descent (경사 하강법)

Gradient Descent는 함수 $J(\theta)$가 연속적으로 미분 가능하다고 가정할 때, 현재 위치 $\theta_{k}$에서의 gradient 벡터 $\nabla J(\theta_{k})$를 활용하여 목적 함수 값을 감소시키는 최적화 기법이다. 함수의 기울기는 해당 점에서 함수 값이 가장 빠르게 증가하는 방향을 나타내므로, 최소화 문제에서는 기울기의 반대 방향으로 이동하게 된다.

$$
\theta_{k+1} = \theta_{k} - \alpha \nabla J(\theta_{k})
$$

여기서, $\alpha > 0$는 학습률(step size)로, 적절한 값 선택이 수렴 속도와 안정성을 결정한다.

```pseudo
Input: 
    - Cost function J(θ)
    - Gradient ∇J(θ)
    - Initial parameter θ₀
    - Learning rate α > 0
    - Max iterations N
    - (Optional) tolerance ε

Initialize:
    θ ← θ₀

for k = 0 to N-1 do
    Compute gradient: g ← ∇J(θ)
    
    if ||g|| < ε then
        break

    Update parameter: θ ← θ - α * g

return θ
```

## 2. Newton's Method

Newton's Method는 목적 함수 $J(\theta)$가 이차 미분 가능하다고 가정할 때, 함수의 2차 도함수(Hessian Matrix) 정보를 반영하여 파라미터를 업데이트하는 함수 최적화 방법이다. 목적 $J(\theta)$를 현재 파라미터 $\theta_k$ 근방에서 2차 테일러 전개를 하면 아래와 같이 근사된다.

$$
J(\theta) \approx J(\theta_k) + \nabla J(\theta_k)^T (\theta - \theta_k) + \frac{1}{2} (\theta - \theta_k)^T \nabla^2 J(\theta_k) (\theta - \theta_k)
$$

이 근사 목적 함수의 stationary point를 구하면 다음과 같은 파라미터 갱신식이 유도된다:

$$
\theta_{k+1} = \theta_k - \left[ \nabla^2 J(\theta_k) \right]^{-1} \nabla J(\theta_k)
$$

여기서, $\nabla J(\theta_k)$는 목적 함수의 gradient 벡터이고, $\nabla^2 J(\theta_k)$는 목적 함수의 Hessian 행렬이다.

Newton's Method는 목적 함수의 이차 미분을 활용하므로, Gradient Descent에 비해 더 빠른 수렴 속도를 가질 수 있지만, 각 단계마다 Hessian의 계산 및 역행렬이 필요하므로 계산 비용이 높고, Hessian matrix가 nonsingular인 것이 요구된다.

```pseudo
Input:
    - Cost function J(θ)
    - Gradient ∇J(θ)
    - Hessian ∇²J(θ)
    - Initial parameter θ₀
    - Max iterations N
    - (Optional) tolerance ε

Initialize:
    θ ← θ₀

for k = 0 to N-1 do
    Compute gradient: g ← ∇J(θ)
    Compute Hessian: H ← ∇²J(θ)

    if ||g|| < ε then
        break

    Update parameter: θ ← θ - H⁻¹ * g

return θ
```

## 3. Gauss-Newton

Gauss-Newton 방법은 비선형 최소제곱 문제를 해결하기 위한 최적화 알고리즘이다. 여기서, 목적 함수는 잔차 함수 $r(\theta)$의 제곱합으로 정의된다.

잔차 함수의 Jacobian 행렬을 $J_r(\theta)$라 하면, gradient와 Hessian은 다음과 같이 근사할 수 있다

$$
\begin{aligned}
\nabla J(\theta) & = J_{r}(\theta)^{T}r(\theta) \\
\nabla^2 J(\theta) & \approx J_r(\theta)^T J_r(\theta)
\end{aligned}
$$
따라서, Gauss-Newton의 파라미터 업데이트는 

$$
\theta_{k+1} = \theta_k - \left(J_r(\theta_k)^T J_r(\theta_k)\right)^{-1} J_r(\theta_k)^T r(\theta_k)
$$
와 같이 표현된다. 이 방식은 Hessian 계산 부담을 줄이면서도, Newton's Method의 장점을 활용할 수 있다.

```pseudo
Input:
    - Residual function vector r(θ)
    - Jacobian matrix J_r(θ)
    - Initial parameter θ₀
    - Max iterations N
    - (Optional) tolerance ε

Initialize:
    θ ← θ₀

for k = 0 to N-1 do
    Compute residual: r ← r(θ)
    Compute Jacobian: J ← J_r(θ)

    if ||Jᵀ r|| < ε then
        break

    Update parameter: θ ← θ - (Jᵀ J)⁻¹ * Jᵀ * r

return θ
```

## 4. Levenberg-Marquardt
마지막 Levenberg-Marquardt 알고리즘은 Gauss-Newton 방법과 Gradient Descent의 장점을 결합한 최적화 기법으로, 특히 비선형 최소제곱 문제에서 효과적이다. 이 알고리즘은 Gauss-Newton 방식의 Hessian 근사에 감쇠항 $\lambda$를 추가해서 업데이트 식을 다음과 같이 수정한다.

$$
\theta_{k+1} = \theta_k - \left(J_r(\theta_k)^T J_r(\theta_k) + \lambda I \right)^{-1} J_r(\theta_k)^T r(\theta_k)
$$

여기서, $I$는 단위 행렬이며, $\lambda > 0$는 감쇠 인자로 알고리즘의 안정성을 조절한다. $\lambda$의 값이 작으면 알고리즘의 Gauss-Newton 방식과 유사하게 작동하고, $\lambda$의 값이 크면 Gradient Descent와 유사한 보폭으로 이동하게 된다. 일반적으로, 목적 함수 값의 감소 정도에 따라 $\lambda$를 동적으로 조정한다.

```pseudo
Input:
    - Residual function vector r(θ)
    - Jacobian matrix J_r(θ)
    - Initial parameter θ₀
    - Initial damping factor λ > 0
    - Max iterations N
    - (Optional) tolerance ε

Initialize:
    θ ← θ₀

for k = 0 to N-1 do
    Compute residual: r ← r(θ)
    Compute Jacobian: J ← J_r(θ)

    if ||Jᵀ r|| < ε then
        break

    Compute Hessian approximation: H ← Jᵀ J
    Update parameter: θ ← θ - (H + λI)⁻¹ * Jᵀ * r

    (Optionally adjust λ based on the reduction in J(θ))

return θ
```

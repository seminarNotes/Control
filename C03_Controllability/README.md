# Controllability
<p align="right">
최초 작성일 : 2024-12-11 / 마지막 수정일 : 2025-01-07
</p>

## 1. Reachability
시스템의 제어 가능성(Controllability)는 시스템을 주어진 시간 내 원하는 상태로 변화시킬 수 있는지 평가하는 개념이다. 먼저, 제어 가능성에 대한 용어를 설명하기 전에 먼저, 도달 가능성(Reachability)에 대해 이해해보자. 연속 상태 방정식이 아래와 같이 주어져 있다고 가정한다.

$$ \dot{x}(t) = Ax(t) + Bu(t) $$

그러면, (우리가 원하는) 특정 상태 $x_{f}$가 도달 가능하다는 것은 제한된 시간 $T_{f}$ 내에, 적당한 입력 변수 $u(t)$가 존재해 아래와 같이 표현된다는 의미이다. 즉, State Transition Integral은 아래와 같이 구해진다.

$$ x_{f} = \int_{0}^{T_{f}} e^{A(T_{f} - s)} Bu(s) ds $$

## 2. Controllability

위 적분식은 주어진 상태 방정식의 닫힌 형식의 해(closed-form solution)이다. 따라서, 위 적분식을 이용해 입력변수 $u$와 상태변수 $x$의 관계를 수학적으로 알 수 있다. 위와 같이 입력변수 $u$가 존재해서, 우리가 원하는 상태변수 $x$를 표현할 수 있다면, 실제로 그 상태는 계산 가능하다고 따라서 (이론상으로) 구현가능하다고 해석할 수 있다. 이를 확장하여, 시스템 자체가 도달 가능하다(reachable)는 것은 모든 점이 도달 가능할 때이고, 시스템이 제어 가능하다(controllable)는 것은 초기 상태변수 $x(0) = x_{0}$와 제한된 시간 $T_{f}$이 주어져서, $x(T_{f}) = 0$을 만드는 적당한 입력 변수 $u$를 계산할 수 있는 시스템을 의미한다.

위 적분식을 적분 사상(선형 사상)으로 표현해보자. 적분 변환 $\Gamma_{T} : U \subset R^{m} \rightarrow X \subset R^{n}$을 통해 입력 변수와 상태 변수의 관계를 아래와 같이 정의할 수 있다. 아래 적분식은 입력에 따른 $T$ 시점의 상태값을 대응하는 사상으로써, 입력-상태 변환(Transformation from Input to State)이다.

$$ x_{T} = \Gamma_{T}(u(s)) = \int_{0}^{T} e^{A(T - s)} Bu(s) ds $$

그러면, 적분 변환은 선형성을 만족하기 때문에 사상 $\Gamma_{T}$은 선형 변환이다. 선형 변환의 image는 벡터 부분 공간을 형성하기 때문에 이러한 벡터 공간을 주어진 시간 $t$에 대한 도달 가능한 상태들의 집합(set of reachable states for a fixed $t$) $R_{t}$라 한다.


$$
R_{t} = ( x(t) | x(t) = \int_{0}^{t} e^{A(t - s)} Bu(s) ds \text{ for some function } u(t) )
$$

적분식 안에 있는 행렬 지수 함수 $e^{At}$은 행렬 $A$에 대해 정의된 지수 함수로, 다음과 같이 무한 급수로 표현된다.

$$ e^{A} = \sum_{k \geq 0} \frac{A^{k}}{k !} $$

하지만, Cayley-Hamilton 정리에 의해, 모든 $n \times n$ 정사각행렬 $A$는 자신의 특성 다항식을 만족하고, 특정 방정식은 n차 다항식이기 때문에, 결과적으로 행렬 지수 함수는 아래와 같이 유한 급수로 표현된다.

$$ e^{A} = \sum_{k = 0}^{n-1} c_{k} A^{k}  $$

그렇다면, 제어 가능하다는 것은 원하는 상태값 $x(T)$가 행렬 $A$의 거듭제곱과 행렬 $B$의 곱으로 표현될 수 있는가에 대한 문제로 귀결되며, 다음과 같은 augmented 행렬로 제어 가능성을 판단한다. 주어진 시스템 $(A, B)$에 대해 제어 가능성 행렬(Controllability matrix)는 아래와 같이 정의한다.

$$ C(A, B) = \left[B \, AB \, A^{2}B \, \cdots \, A^{n-1}B \right] $$

그리고, 이 행렬(선형 사상)의 image를 제어 가능한 부분 집합(Controllable Subspace)라고 하며, $C_{AB}$라 표기한다.

$$ C_{AB} = \text{Image }C(A, B) $$

자명하게, 시스템 $(A, B)$가 제어 가능하다는 것은 제어 가능한 부분 집합 $C_{AB}$가 전체 벡터 공간이라는 것과 동치이며, 아래와 같이 정리할 수 있다.

$$
\begin{align*}
& \text{The system } (A, B) \text{ is Controllable} \\
\textit{if and only if} \quad & \text{Image }\Gamma_{t} = R^{n} \\
\textit{if and only if} \quad & \text{Image }C(A,B) = R^{n} 
\end{align*}
$$


## 3. Controllability Grammian

Controllability Grammian은 제어 이론에서 시스템의 제어 가능성을 정량적으로 분석하는 수학 개념이다. 위에서 소개한 적분 사상의 kernel 함수 $e^{As}B$의 내적의 적분한 값으로 정의된다. 구체적으로, 시스템 $(A,B)$에 대해 finite-time controllability grammian은 아래와 같이 정의된다.

$$
W_{t} = \int_{0}^{t} e^{As} B B^{T} e^{A^{T}s} ds
$$

$W_{t}$은 대칭 행렬이며, positive semi-definite하다. 우리는 앞에서 시스템의 제어 가능성을 부분 공간이 전체 벡터 공간인지를 통해 확인하였다. 그리고, 앞에서 소개한 벡터 부분공간과 Controllability Grammian $W_{t}$의 image는 아래와 같은 관계를 갖는다.

$$
R_{t} = C_{AB} = \text{Image }(W_t)
$$ 

따라서, 위 동치식에 grammian의 image 부분 공간에 대한 동치 조건을 아래와 같이 추가할 수 있으며, grammian을 해로 갖는 방정식으로 시스템 $(A, B)$에 대한 제어 가능성에 대한 필요충분조건을 표현할 수 있다. 시스템  $(A, B)$에 대한 제어 가능할 필요충분조건은 아래 행렬로 표현된 방정식이 유일한 해로 $W$를 갖는다는 것이다.

$$
AW + WA^{T} + BB^{T} = 0
$$

제어가능성에 대한 모든 동치 조건은 아래와 같이 정리된다.

$$
\begin{align*}
& \text{The system } (A, B) \text{ is Controllable} \\
\textit{if and only if} \quad & \text{Image }\Gamma_{t} = R^{n} \\
\textit{if and only if} \quad & \text{Image }C(A,B) = R^{n} \\
\textit{if and only if} \quad & \text{Image }(W_t) = R^{n} \\
\textit{if and only if} \quad & \text{There exists } W \text{ s.t. } AW + WA^{T} + BB^{T} = 0
\end{align*}
$$

이제 grammian을 이용해서 desired state $x_{d}$가 존재할 때, 대응하는 제어 입력 $u(t)$를 설계해본다. reachable을 설명할 때, 입력 변수 $u$와 상태 변수 $x$의 관계를 적분 변환 $\Gamma_{t}$을 통해 아래와 같이 표현했다.

$$ x_{t} = \Gamma_{t}(u(s)) = \int_{0}^{t} e^{A(t - s)} Bu(s) ds $$

finite-time controllability grammian $W_{t}$을 이용해서 우리가 원하는 생태(desired state) $x_d$가 되도록 입력 변수 $u$는 표현할 수 있다. 만약 $u(s) = B^{T}e^{A^{T}(t-s)} W_{t}^{-1} x_d$라 하고, $\Gamma_{t}(u(s))$를 계산해보자.

$$
\begin{align*}
\Gamma_{t}(u(s)) & = \int_{0}^{t} e^{A(t - s)} Bu(s) ds \\
& = \int_{0}^{t} e^{A(t - s)} B B^{T}e^{A^{T}(t-s)} W_{t}^{-1} x_d ds \\
& = \int_{0}^{t} e^{A(t - s)} B B^{T}e^{A^{T}(t-s)}  ds W_{t}^{-1} x_d \\
& = W_{t} W_{t}^{-1} x_d \\
& = I x_d \\
& = x_d
\end{align*}
$$

## 4. Numerical Implementation

바로 위에서 finite-time controllability grammian $W_{t}$을 이용해서 우리가 원하는 생태(desired state) $x_d$가 되도록 입력 변수 $u$는 표현할 수 있다고 했다. 이 부분을 실제 matlab 코드를 이용해서 구현했다. 아래 함수는 시스템 상태 행렬과 상태 변수의 초기값과 목표 상태 변수를 입력 받아 gramian을 계산하였다. 이후, 반복문(for문)을 통해 제어 입력 $u$와 상태 변수 $x$를 시간 구간마다 계산하여, 배열에 추가하였다.

```matlab
% 제어 가능 Grammian 계산
Integrand = @(tau) expm(A * tau) * B * B' * expm(A' * tau); 
W = @(t) integral(Integrand, 0, t, 'ArrayValued', true);
W_final = W(T_final);

% 상태와 입력 계산
for i = 1:sample_size - 1
    U(i) = B' * expm(A' * (T_final - t(i))) * inv(W_final) * x_desired;
    X(:, i + 1) = runge_kutta(A, X(:, i), B, U(i), T_interval);
end
```

마지막으로, 반복문이 완료되면, 배열에 추가된 데이터를 기반으로 결과를 출력 및 시각화하였다. 코드를 실행하는 실행부 파일과 입력값은 아래와 같다.

```matlab
A = [0  1  0;
     0  0  1;
    -2 -3 -4];

B = [0; 0; 1];
C = [1 0 0];
D = 0;

x_initial = [1; 0; 0];
x_desired = [3; 2; 0.4];

simulation_controllability_gramian(A, B, C, D, x_initial, x_desired);
```

사용한 상태 행렬들과 초기값, 목표값은 물리적 의미가 있는 것이 아닌 임의로 값을 사용했다. order == 3일 때, 실행한 후, 터미널에 출력된 결과는 아래와 같다. 타겟이 되는 상태 변수는 $x_{desired} = (3, 2, 0.4)$이고, 실제 controllability gramian $W_{t}$을 사용해 제어한 상태 변수는 $x_{reached} = (2.9006, 1.9823, 0.3197)$이며, 이 둘의 차이는 $(0.0994, 0.0177, 0.0803)$으로 상당히 높은 정확도로 수렴하였음을 확인할 수 있다.
    
```matlab
>> run_simulation_controllaility_gramian

x_desired = 
    3.0000
    2.0000
    0.4000

 x_reached = 
    2.9006
    1.9823
    0.3197

x_desired - x_reached = 
    0.0994
    0.0177
    0.0803
```

또, 상태 변수가 수렴을 하는 과정에서의 상태 변수와 제어 입력의 궤적은 아래와 같이 표현된다.

<img src="https://github.com/seminarNotes/studyNotes/blob/main/C03_Controllability/order2_input.jpg" alt="Controllability Order3 State" width="500">

<img src="https://github.com/seminarNotes/studyNotes/blob/main/C03_Controllability/order3_input.jpg.jpg" alt="Controllability Order3 Input" width="500">


## 5. Minimum Energy Ellipsoid

제어 이론에서 시스템 성능을 평가하는 중요한 기준 중 하나는 입력 에너지의 효율성이다. 제어가 가능하더라도 최소한의 에너지를 사용하여 시스템을 원하는 상태나 출력으로 제어하는 것이 이상적이다. 이 개념은 최적 제어(Optimal Control) 분야에서 주로 다루는 주제이지만, Controllable Grammian을 활용해 아래와 같이 설명할 수 있다. Grammian을 이용하면, 시스템이 특정 상태에 도달하기 위해 필요한 최소 에너지를 계산하고, 이를 기반으로 효율적인 입력 설계를 수행할 수 있다

$x_d$에 도달할 수 있게 하는 입력 변수 $u(t)$은 $u(s) = B^{T}e^{A^{T}(t-s)} W_{t}^{-1} x_d$로 주어지며, 특정 상태로 $x_{d}$로 시스템을 이동시키기 위해 필요한 입력을 의미한다. 여기서 제어 입력 $u(t)$의 $L2$-norm은 에너지 소비를 측정하는 기준으로 사용된다. 이제 제어 입력 $u(t)$의 소비되는 에너지를 계산해보자.

$$
\begin{align*}
\Vert u \Vert_{L2}^{2} & = \int_0^t u(s)^2 ds \\ 
& = \int_0^t x_d^T W_{T_f}^{-1} e^{A(T_f-s)} BB^T e^{A^T(T_f-s)} W_{T_f}^{-1} x_d ds \\
& = x_d^T W_{T_f}^{-1} \left( \int_0^t e^{A(T_f-s)} BB^T e^{A^T(T_f-s)} ds \right) W_{T_f}^{-1} x_d \\
& = x_d^T W_{T_f}^{-1} W_{T_f} W_{T_f}^{-1} x_d \\
& = x_d^T W_{T_f}^{-1} x_d
\end{align*}
$$

만약에 제어 입력 $u(t)$의 에너지가 아래와 같은 조건을 만족한다고 가정하자.

$$
\Vert u \Vert_{L2}^{2} = x^T W_{T_f}^{-1} x \leq 1
$$

이 조건은 입력 에너지가 $1$ 이하로 제한되었을 때, $x_{d}$에 도달할 수 있는 상태들을 정의한다. 이 상태들의 집합은 최소 에너지 타원체(Minimum Energy Ellipsoid)로 표현된다.

$$
\text{Ellipsoid} = ( x \in R^{n} | x^{T}W_{T}^{-1}x \leq 1 )
$$

여기서, 집합을 Ellipsoid라 부르는 이유는 상태 벡터들이 이차 형식(quadratic form)의 부등식으로 조건이 되어 있기 때문에 이 영역은 타원형으로 표현되기 때문이다. 이 타원체의 모양은 controllable Gramian $W_{T}$에 의해 결정되는데, 타원체의 모양을 결정 짓는 축의 길이는 $W_{T}$의 $i$번째 eigenvalue의 제곱근 $\sqrt{\lambda_{i}(W_{T})}$에 의해 결정되고, 축의 방향은 대응하는 $i$번째 eigenvector의 방향에 의해 결정된다.

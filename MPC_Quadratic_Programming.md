# MPC Qudratic Programming
<p align="right">
최초 작성일 : 2025-01-06 / 마지막 수정일 : 2025-01-11
</p>

## 1. Model Predictive Control 의미

Model Predictive Control, MPC은 연구 분야와 산업 분야에서 가장 널리 사용되는 현대 제어 방법이다. MPC는 주어진 조건과 목적을 달성하면서 최소한의 비용을 계산하여 주기적으로 입력값을 계산한다는 점에서 시스템의 성능을 최적화할 수 있다. 또한, 제약 조건을 명시적으로 처리할 수 있으며,  단일 입력-출력(SISO) 시스템부터 다중 입력-출력(MIMO) 시스템까지 유연하게 시스템에 적용할 수 있다는 장점이 있다. 이러한 장점과 함께 MPC는 참조 상태와의 차이와 제어 입력을 최소한으로 하여 원하는 궤적을 따라 시스템이 추적할 수 있도록 하는 작업을 수행할 수 있기 때문에 주로 자율 운행 차량이나 반복 잡업을 수행하는 로봇 분야 등에 적용될 수 있다.

아래 글에서는 MPC의 정의를 소개하고 LQ로 변환하는 부분을 설명하고 있다.

MPC는 현재 상태에서 시스템의 미래 동작을 예측하고, 주어진 제약 조건을 만족하며 최적화된 제어 입력을 계산하는 제어 기법이다. 주어진 시스템에서 MPC는 상태 $x_{t}$와 제어 입력 $u_{t}$가 다음의 선형 시스템 동역학에 따라 움직인다고 가정하자.

$$
x_{t+1} = Ax_{t} + Bu_{t}
$$

또한, MPC는 아래와 같은 제약 조건(Constraints)를 포함한다. 이 때, 아래의 식을 상태 변수에 대한 제약 조건이라 하며, 상수가 아닌 인덱스 $i$에 대한 변수로 주어질 수도 있으며, 상태 변수에 대한 제약 조건은 시스템에 따라 물리적으로 구현될 수 없는 상태가 존재하기 때문에 이를 수학적으로 묘사한 제약 조건이다.

$$
x_{min} \leq x_{t+i} \leq x_{max} \quad \text{for } 1 \leq i \leq N 
$$

마찬가지로 특정 범위 내에서만 입력할 수 있는 제어 입력 $u_{t}$은 아래와 같은 부등식을 만족한다.

$$
u_{min} \leq u_{t+i} \leq u_{max} \quad \text{for } 0 \leq i \leq N-1
$$

매 시간 스텝 $t$마다, 시간 구간 $[t, t+N]$에서 위 시스템과 제약 조건을 만족하며, 아래 비용 함수 $J$를 최소화하는 최적 제어 입력 시퀀스 $u_{t:t+N-1}^{*}$를 구하는 것이 바로 MPC 문제이다.

$$
\min_{u_{t:t+N-1}} J = \sum_{i=0}^{N-1} x_{t+i}^T Q x_{t+i} + u_{t+i}^T R u_{t+i} + x_{t+N}^T P x_{t+N}
$$


## 2. Quadratic Program 변환

아래에서는 주어진 MPC 문제를 LQ로 변환하는 과정을 정리하였다. 먼저 앞에서 소개한 MPC는 아래와 같은 최적 문제를 의미한다.

$$
\begin{align*}
\min_{u_{t:t+N-1}} J & = \sum_{i=0}^{N-1} x_{t+i}^T Q x_{t+i} + u_{t+i}^T R u_{t+i} + x_{t+N}^T P x_{t+N} \\
& \text{subject to } \\
\text{state equation}& x_{t+1} = Ax_{t} + Bu_{t} \\ 
\text{current state}& x_{t} \text{ is given} \\
\text{state constraints}& x_{min} \leq x_{t+i} \leq x_{max} \quad \text{for } 1 \leq i \leq N \\
\text{input constraints}& u_{min} \leq u_{t+i} \leq u_{max} \quad \text{for } 0 \leq i \leq N-1
\end{align*}
$$


해당 최적화 문제를 풀고 난 후, 최적 제어 입력 시퀀스 $u_{t:t+N-1}^{*}$에서 첫 번째 항만을 사용한다. 이러한 MPC 문제를 해결하는 방법 중 하나는 quadratic problem(QP)으로 문제를 변환하여 해결하는 방법이 있다.

$$
\begin{align*}
x_{t+1} & = A x_{t} + B u_{t} \\
x_{t+2} & = A x_{t+1} + B u_{t+1} = A^2 x_{t} + A B u_{t} + B u_{t+1} \\
& \vdots  \\
x_{t+N} & = A^N x_{t} + A^{N-1} B u_{t} + \cdots + A B u_{t+N-2} + B u_{t+N-1}
\end{align*}
$$

그러면, 아래와 같이 행렬 형식으로 표현할 수 있다.

$$
\begin{bmatrix}
x_{t+1} \\
x_{t+2} \\
x_{t+3} \\
\vdots \\
x_{t+N}
\end{bmatrix} =
\begin{bmatrix}
A \\
A^2 \\
A^3 \\
\vdots \\
A^N
\end{bmatrix}
x_t
+
\begin{bmatrix}
B & 0 & 0 & \cdots & 0 \\
AB & B & 0 & \cdots & 0 \\
A^2B & AB & B & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
A^{N-1}B & A^{N-2}B & A^{N-3}B & \cdots & B
\end{bmatrix}
\begin{bmatrix}
u_t \\
u_{t+1} \\
u_{t+2} \\
\vdots \\
u_{t+N-1}
\end{bmatrix}
$$

이제 위 행렬을 아래와 같이 정리 할 수 있다.

$$
\begin{align}
x_{t+1:t+N} = M x_{t} + Su_{t:t+N-1}
\end{align}
$$

여기서,

$$
M =
\begin{bmatrix}
A \\
A^2 \\
A^3 \\
\vdots \\
A^N,
\end{bmatrix},
\quad
S =
\begin{bmatrix}
B & 0 & 0 & \cdots & 0 \\
AB & B & 0 & \cdots & 0 \\
A^2B & AB & B & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
A^{N-1}B & A^{N-2}B & A^{N-3}B & \cdots & B
\end{bmatrix}
$$


$$
\begin{align*}
J & = \sum_{i=0}^{N-1} x_{k}^{T}Qx_{k} + u_{k}^{T}Ru_{k} + x_{N}^{T}Px_{N} \\
& = x_t^T Q x_t + 
\begin{bmatrix}
x_{t+1} \\
x_{t+2} \\
\vdots \\
x_{t+N}
\end{bmatrix}
^{T}
\begin{bmatrix}
Q & 0 & \cdots & 0 \\
0 & Q & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & Q_f
\end{bmatrix}
\begin{bmatrix}
x_{t+1} \\
x_{t+2} \\
\vdots \\
x_{t+N}
\end{bmatrix}
+ 
\begin{bmatrix}
u_t \\
u_{t+1} \\
\vdots \\
u_{t+N-1}
\end{bmatrix}
^{T}
\begin{bmatrix}
R & 0 & \cdots & 0 \\
0 & R & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & R
\end{bmatrix}
\begin{bmatrix}
u_t \\
u_{t+1} \\
\vdots \\
u_{t+N-1}
\end{bmatrix} \\
& = x_t^T Q x_t + x_{t+1:t+N}^T \bar{Q} x_{t+1:t+N} + u_{t:t+N-1}^T \bar{R} u_{t:t+N-1}
\end{align*}
$$

이제 비용 함수를 행렬 형식으로 표현하기 위해 이차 형식을 아래와 같이 행렬 벡터 곱으로 표현한다.

$$
\begin{align}
J = x_t^T Q x_t + x_{t+1:t+N}^T \bar{Q} x_{t+1:t+N} + u_{t:t+N-1}^T \bar{R} u_{t:t+N-1}
\end{align}
$$

여기서, 

$$
\bar{Q} =
\begin{bmatrix}
Q & 0 & \cdots & 0 \\
0 & Q & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & P
\end{bmatrix}, \quad
\bar{R} =
\begin{bmatrix}
R & 0 & \cdots & 0 \\
0 & R & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & R
\end{bmatrix}
$$

이제 행렬 형태가 된 비용 함수 (2)에 다가 행렬 형태가 된 상태 방정식 (1)을 대입하면, 변수 $U$에 대한 2차 방정식(quadratic polynomial)이 유도된다.

$$
\begin{align*}
J & = x_t^T Q x_t + x_{t+1:t+N}^T \bar{Q} x_{t+1:t+N} + u_{t:t+N-1}^T \bar{R} u_{t:t+N-1} \\
& = x_t^T Q x_t + (M x_{t} + Su_{t:t+N-1})^T \bar{Q} (M x_{t} + Su_{t:t+N-1}) + u_{t:t+N-1}^T \bar{R} u_{t:t+N-1} \\
& = u_{t:t+N-1}^{T}(\bar{R} + S^{T} \bar{Q} S) u_{t:t+N-1} + 2 x_{t}^{T}M^{T}\bar{Q}S u_{t:t+N-1} + x_{t}^{T}(Q+M^{T}\bar{Q}M)x_{t} \\
& = U^{T}HU + 2q^{T}U + c
\end{align*}
$$

여기서, 

$$
\begin{align*}
U & = u_{t:t+N-1}, \\
H & = \bar{R} + S^{T} \bar{Q} S, \\
q & = S^{T} \bar{Q}Mx_{t}, \\
c & = x_{t}^{T}(Q+M^{T}\bar{Q}M)x_{t}
\end{align*}
$$

이제 주어진 제약 조건(constraints)를 변수 $U$에 대한 행렬 부등식으로 표현한다. 

먼저, 

$$
U_{min} = 
\begin{bmatrix}
u_{min} \\
u_{min} \\
u_{min} \\
\vdots \\
u_{min} \\
\end{bmatrix}
\quad
U_{max} = 
\begin{bmatrix}
u_{max} \\
u_{max} \\
u_{max} \\
\vdots \\
u_{max} \\
\end{bmatrix}
\quad
X_{min} = 
\begin{bmatrix}
x_{min} \\
x_{min} \\
x_{min} \\
\vdots \\
x_{min} \\
\end{bmatrix}
\quad
X_{max} = 
\begin{bmatrix}
x_{max} \\
x_{max} \\
x_{max} \\
\vdots \\
x_{max} \\
\end{bmatrix}
\quad
$$

라 두면, 제약 조건

$$
\begin{align*}
& x_{min} \leq x_{t+i} \leq x_{max} \quad \text{for } 1 \leq i \leq N \\
& u_{min} \leq u_{t+i} \leq u_{max} \quad \text{for } 0 \leq i \leq N-1
\end{align*}
$$

은 아래와 같이 행렬 형식으로 표현된다.

$$
\begin{align*}
& X_{min} \leq M x_{t} + SU \leq X_{max} \\
& U_{min} \leq U \leq U_{max}
\end{align*}
$$

그러면, 각각 아래와 같이 행렬 부등식으로 표현할 수 있다.

$$
\begin{align*}
&
\begin{bmatrix}
S \\
-S
\end{bmatrix}
U \leq
\begin{bmatrix}
X_{max} - M x_{t} \\
-X_{min} + M x_{t}
\end{bmatrix}
\\
&
\begin{bmatrix}
I \\
-I
\end{bmatrix}
U \leq
\begin{bmatrix}
U_{max} \\ - U_{min}
\end{bmatrix}
\end{align*}
$$

이제 최종적으로,

$$
G =
\begin{bmatrix}
S \\
-S \\
I \\
-I
\end{bmatrix}, \quad
W =
\begin{bmatrix}
X_{\text{max}} \\
-X_{\text{min}} \\
U_{\text{max}} \\
-U_{\text{min}}
\end{bmatrix}, \quad
T =
\begin{bmatrix}
-M \\
M \\
0 \\
0
\end{bmatrix}
$$

이라 쓰면, 최종적인 augmented matrix 부등식을 얻을 수 있다.

$$
\begin{align}
GU \leq W + T x_{t}
\end{align}
$$

위에서 행렬 형태로 유도된 상태 방정식 , 제약 조건 , 비용 함수를 정리하여 최적화 문제를 정리하면 아래와 같다.

$$
\begin{align*}
\min_{U} J & = x_t^T Q x_t + x_{t+1:t+N}^T \bar{Q} x_{t+1:t+N} + u_{t:t+N-1}^T \bar{R} u_{t:t+N-1} \\
& \text{subject to } \\
\text{state equation}& x_{t+1:t+N} = M x_{t} + SU \\
\text{constraints}& GU \leq W + T x_{t} \\
\end{align*}
$$

## 3. 시뮬레이션  

위에서 일반적인 MPC 문제를 quadratic programming 문제로 변환하였다. 이제 적당한 시스템 $(A,B,C,D)$에 대해서 quadratic programming을 시뮬레이션해보자. 먼저 다음과 같은 시스템을 가정한다.

$$
A = \begin{bmatrix}
2.4936 & -1.2130 & 0.4176 \\
2 & 0 & 0 \\
0 & 1 & 0
\end{bmatrix}, \quad
B = \begin{bmatrix}
0.25 \\
0 \\
0
\end{bmatrix}, \quad
C = \begin{bmatrix}
0.0684 & 0.1288 & 0.0313
\end{bmatrix}, \quad
D = 0
$$

시뮬레이션에서 필요한 파라미터인 예측 구간 크기, 초기 상태값, 전체 시간 구간을 정의한다.

``` matlab
N_prediction = 10;      % 예측 구간 크기
x_initial = [1; 1; 1];  % 초기 상태값
T_final = 50;           % 전체 시간 구간
```

이제 기존 MPC 비용 함수가 아래와 같이 주어져 있을 때,

$$
J & = \sum_{i=0}^{N-1} x_{t+i}^T Q x_{t+i} + u_{t+i}^T R u_{t+i} + x_{t+N}^T P x_{t+N}
$$

상태 변수, 제어 입력, 종단 조건에 대한 weight 행렬 $Q$, $R$, $P$를 각각 아래와 같이 가정한다.

``` matlab
Q = C' * C;             
R = eye(size(B, 2));    
P = Q;
```

이제 QP 문제에서 사용되는 행렬 $M$, $S$, $\bar{Q}$, $\bar{R}$를 아래의 수식에 따라 차례대로 선언한다.

$$
M =
\begin{bmatrix}
A \\
A^2 \\
A^3 \\
\vdots \\
A^N,
\end{bmatrix},
\quad
S =
\begin{bmatrix}
B & 0 & 0 & \cdots & 0 \\
AB & B & 0 & \cdots & 0 \\
A^2B & AB & B & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
A^{N-1}B & A^{N-2}B & A^{N-3}B & \cdots & B
\end{bmatrix}
$$

를 각각 아래와 같이 반복문을 통해 정의한다.

``` matlab
M = [];
S = [];
for idx1 = 1:N_prediction
    M = [M; A^idx1];
    temp = [];
    for idx2 = 1:idx1
        temp = [temp A^(idx1 - idx2) * B];
    end
    temp = [temp zeros(size(temp, 1), (N_prediction - idx1) * size(B, 2))];
    S = [S; temp];
end
```

또, augmented matrics인 
$$
\bar{Q} =
\begin{bmatrix}
Q & 0 & \cdots & 0 \\
0 & Q & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & P
\end{bmatrix}, \quad
\bar{R} =
\begin{bmatrix}
R & 0 & \cdots & 0 \\
0 & R & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & R
\end{bmatrix}
$$

는 각각 아래와 같이 내장함수를 이용할 수 있다.

``` matlab
Q_hat = blkdiag(kron(eye(N_prediction - 1), Q), P);
R_hat = kron(eye(N_prediction), R);
```

이제 QP문제에서 비용 함수

$$
U^{T}HU + 2q^{T}U + c
$$

에서 행렬 $H$와 $q$를 정의한다. 이 때, 상수항인 $c$는 최적 입력을 결정하는데 영향을 주지 않기 때문에 두 행렬 행렬 $H$와 $q$만 정의하고, 아래와 같은 최적화 문제를 해결한 후, 상수항 $c$를 더해주는 것으로 바꾸어 생각한다.

$$
\min_{U} U^{T}HU + 2q^{T}U
$$

아래에서는 $H$를 대칭화를 통해 재정의한다.

``` matlab
H = S' * Q_hat * S + R_hat; 
H = (H + H') / 2; % 대칭 행렬 보장
q = S' * Q_hat * M;
```

이제 위에서 정의한 변수를 이용해서 QP문제에서 최적화 문제를 표현하고, 함수 quadprog를 이용해서 최적화 문제를 해결한다. 이후, 최적 입력 벡터가 계산되면, 첫번째 성분과 상태 방정식을 이용해서 상태 변수와 출력 변수를 계산한다. 이를 통해, 입력값 $u$, 상태값 $x$, 출력값 $y$을 각각 업데이트를 하면서 시뮬레이션을 수행한다.

``` matlab
for k = 1:T_final
    % 비용 함수의 선형항 계산
    f = q * x(:, k); 
    
    % QP 문제 풀이
    u_mpc = quadprog(H, f, [], [], [], [], ...
        -0.25 * ones(N_prediction, 1), 0.25 * ones(N_prediction, 1));
    
    % 제어 입력 사용 및 상태 업데이트
    u(k) = u_mpc(1); % 첫번째 성분만 사용
    x(:, k + 1) = A * x(:, k) + B * u(k); 
    y(:, k + 1) = C * x(:, k + 1); 
end
```







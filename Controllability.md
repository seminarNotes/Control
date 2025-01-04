# Controllability
<p align="right">
최초 작성일 : 2024-10-13 / 마지막 수정일 : 2025-01-03
</p>

## 1. Controllability의 의미

선형 시간불변 시스템(Linear Time Invariant System)은 아래 연립 미분방정식으로 주어진다.

$$\dot{x}(t) = Ax(t) + Bu(t)$$
$$y(t) = Cx(t) + Du(t)$$

행렬 $A$는 상태 행렬, $B$는 입력 행렬, $C$는 출력 행렬, $D$는 직접 전달 행렬, $x(t)$는 상태 변수, $y(t)$는 출력 변수이다. 그러면, 행렬 연산을 통해 입력과 출력의 관계를 결정하는 전달함수 $G(s)$는 상태 방정식으로부터 아래와 같이 유도된다.

$$G(s) = C(sI - A)^{-1}B + D$$

여기서, 전달 함수 $G(s)$의 극점은 특성 방정식  $\text{det}(sI - A)$의 해와 동일하다. 따라서, 행렬 $A$의 eigenvalue에 의해 전달 함수 $G(s)$의 특성 방정식이 결정된다. 위 사실을 이용해서 이루고자 하는 목표는 주어진 전달 함수 $G(s)$를 원하는 전달 함수 $G_{\text{goal}}(s)$로 변환하여, 시스템을 제어하는 것이다. 제어기를 설계하는 주된 과정은 아래와 같다.

먼저, 주어진 전달 함수 $G(s)$으로부터 상태 방정식을 유도한다.

$$\dot{x}(t) = Ax(t) + Bu(t)$$

전달 함수 $G(s)$를 변환하기 위해서는 행렬 $A$를 업데이트 해야 한다. 입력 변수 $u(t)$를 적절하게 조절하여, 아래와 같이 피드백을 반영한다.

$$u(t) = -Kx(t) + r(t)$$

여기서, $-K$는 음성 피드백 행렬이고, $r(t)$은 새로운 기준 입력 변수이다. 제어 입력 $u(t)$를 입력하면, 아래 식을 얻는다.

$$\dot{x}(t) = (A-BK)x(t) + Br(t)$$

따라서, 행렬 $A-BK$의 특정 방정식이 원하는 전달 함수 $G_{\text{goal}}(s)$의 분모와 동일하게 만드는 행렬 $K$를 찾는 것이 제어기 설계의 핵심이다. 원하는 전달 함수라는 것은 시스템 복잡도나 주어진 문제에 따라 다양하게 설정할 수 있으며, 예를 들어, PID 제어기는 간단한 시스템에서 주로 사용되며, LQR 제어기는 최적 제어 문제에 적합하다. MPC 제어기는 복잡한 제약 조건이 있는 시스템을 제어할 때 유리하다.

## 2. Controllability의 조건
모든 시스템이 Controllability하진 않고, 주어진 상태 방정식

$$
\dot{x}(t) = (A - KB)x(t) + Br(t)
$$

가 제어 가능하기 위한 필요충분조건은, 크기 $n \times n$ 정방행렬 $A$ 와 $n \times 1$ 벡터 $B$에 대해 행렬이 full rank이다.

$$
\text{rank}
\begin{bmatrix}
B & AB & A^2B & \cdots & A^{n-1}B
\end{bmatrix}
= n
$$

## 3. Controllability 예제

아래와 같이 상태방정식이 주어졌다고 가정하자.

$$
\dot{x}(t) = 
\begin{bmatrix}
-2 & 0 \\
0 & -3
\end{bmatrix}
x(t) +
\begin{bmatrix}
1 \\
1
\end{bmatrix}
u(t)
$$

$$
y(t) = 
\begin{bmatrix}
1 & 2
\end{bmatrix}
x(t)
$$

그러면, 아래 제어 가능성 행렬은 invertible하므로, full rank이다.

$$
[A AB] =
\begin{bmatrix}
1 & -2 \\
1 & -3
\end{bmatrix}
$$

이제 위 상태 방정식에 대한 특정 방정식을 아래 식으로 변경해야한다고 가정하자.

$$
s^{2} + 10 s + 1 = 0
$$

피드백 행렬 $K$에 대해 제어 입력 $u(t)=-Kx(t)+r(t)$이라 하면, 아래와 같이 상태 방정식이 변환된다.

$$
\dot{x}(t) = 
\begin{bmatrix}
-2-k_{1} & -k_{2} \\
-k_{1} & -3-k_{2}
\end{bmatrix}
x(t) +
\begin{bmatrix}
1 \\
1
\end{bmatrix}
r(t)
$$

이제 변환된 상태 방정식의 특성 방정식을 계산하면,

$$
\text{det}
(sI-
\begin{bmatrix}
-2-k_{1} & -k_{2} \\
-k_{1} & -3-k_{2}
\end{bmatrix}
) = (s+2+k_{1})(s+3+k_{2})-k_{1}k_{2}
$$

을 얻고, 특성 방정식을 $s^{2} + 10 s + 1 = 0$와 계수 비교를 통해 아래와 같이 $(k_{1}, k_{2})$ 를 계산한다.

$$
(k_{1}, k_{2} = (-15, 20)
$$

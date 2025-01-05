# Observerability
<p align="right">
최초 작성일 : 2024-10-13 / 마지막 수정일 : 2025-01-05
</p>

## 1. Observerability의 의미

제어 공학에서 시스템의 상태를 실시간으로 추정하기 위해 옵저버(Observer)를 설계할 수 있다. 옵저버를 통해 시스템의 상태 변수 중, 일부는 직접 측정할 수 없는 상황에서 시스템의 출력 값과 입력 값을 기반으로 내부 상태를 추정할 수 있다. 특히, 상태 피드백 제어기 설계 시, 제어 입력 값을 계산하는 경우, 시스템의 상태가 필요하지만, 직접 측정할 수 없는 경우 옵저버를 통해 상태값을 제공하여 제어기를 설계할 수 있다.

먼저 아래와 같이 LTI 시스템이 주어졌다고 가정하자.

$$
\dot{x}(t) = Ax(t) + bu(t), \quad y(t) = cx(t)
$$

만약, 실제 상태 변수 $x(t)$와 구분하기 위해 측정된 혹은 추정된 값을 $\hat{x}(t)$이라 쓴다면, 옵저버는 추정된 상태가 실제 상태와 가까워지도록, 아래와 같이 상태 행렬 $A$를 수정하기 위해 아래와 같이 가정한다.

$$
\dot{\hat{x}}(t) = A\hat{x}(t) + bu(t) + L\big(y(t) - \hat{y}(t)\big)
$$

$$
\hat{y}(t) = c\hat{x}(t)
$$

행렬 $L$은 옵저버의 gain으로, 최종적으로 상태 추정 오차는 아래와 같이 정의한다.

$$
e(t) = x(t) - \hat{x}(t), \quad \dot{e}(t) = \dot{x}(t) - \dot{\hat{x}}(t) = (A - Lc)e(t)
$$

옵저버의 목표는 상태 추정 오차가 시간이 지남에 따라 0에 가까워 지도록 만들어, 실제 상태 변수 $x(t)$와 추정 상태 변수 $\hat{x}(t)$의 차이를 0으로 만드는 것이다. 이를 위해, Error Dynamics인 위 식에서, 행렬 $A-LC$의 eigenvalue가 복소 평면의 왼쪽 방평면(half-plane)에 위치 해야 한다. 이 때, 행렬 $A$와 벡터 $C$는 시스템에 의해 결정되기 때문에 결국, gain $L$을 적절하게 선택하는 문제로 정리된다.

## 2. Observerability의 조건

controllability와 마찬가지로, 모든 시스템의 옵저버를 유도할 수 있는 것이 아니다. 옵저버를 유도할 수 있는 시스템을 completely obserability라고 하는데, 이 조건은 주어진 상태 방정식

$$
\dot{x}(t) = Ax(t) + Bu(t), \quad y(t) = Cx(t)
$$

가 관측 가능하기 위한 필요충분조건은, 크기 $n \times n$ 정방행렬 $A$와 $1 \times n$ 행 벡터 $C$에 대해 행렬이 full rank이다.

$$
\text{rank} 
\begin{bmatrix}
C \\
CA \\
CA^2 \\
\vdots \\
CA^{n-1}
\end{bmatrix}
= n
$$

## 3. Observerability의 예제

아래와 같이 상태 방정식이 주어졌다고 가정하자.

$$
A = \begin{bmatrix}
0 & 1 \\
0 & -1
\end{bmatrix}, \quad
b = \begin{bmatrix}
0 \\
1
\end{bmatrix}, \quad
c = \begin{bmatrix}
1 & 0
\end{bmatrix}
$$

그러면,

$$
\begin{bmatrix}
C \\
CA
\end{bmatrix}
= \begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

이기 때문에, invertible하고, full rank이다. 따라서, 주어진 시스템은 completely observerable(C.O.)하다.

이제, 아래 특성 방정식과 일치하도록 옵저버를 디자인한다. 이 말은 옵저버 gain $L$을 조정하여 $A-LC$ 행렬의 고유값을 타겟 특정 방정식의 근으로 설명하여, 상태 오차가 원하는 속도로 소멸되는 것을 보장하도록 옵저버를 설계한다는 의미이다. 따라서, 추정 상태가 실제 상태를 추적하게 된다. 타겟 특성 방정식은 아래와 같다.

$$
s^2 + 2\sqrt{3}s + 6 = 0
$$

옵저버를 유도하기 위해 벡터 $L = (l_1, l_2)^T$를 가정한다.  그러면,

$$
A - Lc = \begin{bmatrix}
0 & 1 \\
0 & -1
\end{bmatrix} - \begin{bmatrix}
l_1 \\
l_2
\end{bmatrix}
\begin{bmatrix}
1 & 0
\end{bmatrix}
= \begin{bmatrix}
-l_1 & 1 \\
-l_2 & -1
\end{bmatrix}
$$

이제 위 행렬의 특성 방정식을 계산하면,

$$
\text{det}(sI - (A - Lc)) =
\text{det}(
sI - \begin{bmatrix}
-l_1 & 1 \\
-l_2 & -1
\end{bmatrix}
) = \text{det} \begin{bmatrix}
s + l_1 & -1 \\
l_2 & s + 1
\end{bmatrix}
$$

$$
(s + l_1)(s + 1) + l_2 = s^2 + (l_1 + 1)s + (l_1 + l_2)
$$

위 특정 방정식을 타겟 특정 방정식 $s^2 + 2\sqrt{3}s + 6 = 0$와 계수를 비교하여, $l_{1} =  2\sqrt{3} - 1$과 $l_{2} = 7 - 2\sqrt{3}$를 계산한다. 이를 통해 옵저버 gain $L$을 아래와 같이 얻는다.

$$
L = \begin{bmatrix}
2\sqrt{3} - 1 \\
7 - 2\sqrt{3}
\end{bmatrix}
$$

clear all
clc
global A B C L

% 시스템 행렬 정의
A = [0 1; 0 -1];
B = [0; 1];
C = [1 0];
l1 = 2*sqrt(3) - 1;
l2 = 7 - 2*sqrt(3);

% 초기 상태 및 설정
X(:,1) = [1; 0];
Xhat(:,1) = [0; 0];
Tf = 20;           % 시뮬레이션 종료 시간
Ti = 0.01;         % 샘플링 시간
t = 0:Ti:Tf;       % 시간 벡터
sample_size = size(t,2);
Ut = [];           % 제어 입력 저장소 초기화
L = [l1; l2];      % 옵저버 게인

% 시뮬레이션 루프
for i = 1:sample_size-1
    U = sin(2*pi*0.001*i);                 % 제어 입력
    y(i) = C * X(:,i);                     % 실제 출력
    yhat(i) = C * Xhat(:,i);               % 옵저버 출력
    e(i) = y(i) - yhat(i);                 % 출력 오차
    
    % Runge-Kutta 방식으로 상태 업데이트
    X(:,i+1) = rk_motor(X(:,i), U, Ti);
    Xhat(:,i+1) = rk_observer(Xhat(:,i), U, Ti, e(i));
    Ut = [Ut, U];                          % 제어 입력 저장
end

% 결과 플롯
figure(1)
plot(t, X(1,:));
title('State trajectories');

figure(2)
plot(t(1:sample_size-1), Ut);
title('Control input');

figure(3)
plot(t(1:sample_size-1), e);
title('Estimation error');

figure(4)
plot(t(1:sample_size-1), y, 'DisplayName', 'Actual Output');
hold on
plot(t(1:sample_size-1), yhat, 'DisplayName', 'Estimated Output');
title('Motor position estimation');
legend;

figure(5)
plot(t, X(2,:), 'DisplayName', 'Actual Velocity');
hold on
plot(t, Xhat(2,:), 'DisplayName', 'Estimated Velocity');
title('Motor velocity estimation');
legend;

% Runge-Kutta 방식 함수들
function DX = rk_motor(X, U, T)
    k1 = motor(X, U) * T;
    k2 = motor(X + k1 * 0.5, U) * T;
    k3 = motor(X + k2 * 0.5, U) * T;
    k4 = motor(X + k3, U) * T;
    DX = X + ((k1 + k4) / 6.0 + (k2 + k3) / 3.0);
end

function DX = rk_observer(X, U, T, e)
    k1 = motor_ob(X, U, e) * T;
    k2 = motor_ob(X + k1 * 0.5, U, e) * T;
    k3 = motor_ob(X + k2 * 0.5, U, e) * T;
    k4 = motor_ob(X + k3, U, e) * T;
    DX = X + ((k1 + k4) / 6.0 + (k2 + k3) / 3.0);
end

function dx = motor(x, u)
    global A B
    dx = A * x + B * u;
end

function dx = motor_ob(x, u, e)
    global A B L
    dx = A * x + B * u + L * e;
end

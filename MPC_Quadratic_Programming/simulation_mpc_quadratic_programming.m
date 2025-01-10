function simulation_mpc_quadratic_programming(A, B, C, D)

% 예측 구간, 초기 상태, 시간 크기 정의
N_prediction = 10;      
x_initial = [1; 1; 1]; 
T_final = 50;

% 입출력/상태 변수 초기화
u = zeros(1, T_final); 
x = zeros(size(A, 1), T_final + 1); 
x(:, 1) = x_initial; 
y = zeros(size(C, 1), T_final + 1);
y(:, 1) = C * x_initial; 

% Cost 함수에 포함되는 weight matrices
Q = C' * C;             % state weight
R = eye(size(B, 2));    % input weight
P = Q;                  % terminal state cost

% QP문제 상태 방정식에서 행렬 M, S 정의
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

% QP문제 비용 함수에서 행렬 Q_hat, R_hat 정의
Q_hat = blkdiag(kron(eye(N_prediction - 1), Q), P);
R_hat = kron(eye(N_prediction), R);

% 이차 형식을 위한 행렬 H 정의
H = S' * Q_hat * S + R_hat; 
H = (H + H') / 2; % 대칭 행렬 보장
q = S' * Q_hat * M;


% Simulation(반복문)
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

% Plot results
figure;

% Plot state trajectories
subplot(3, 1, 1);
plot(0:T_final, x(1, :), '-o', 'DisplayName', 'x_1');
hold on;
plot(0:T_final, x(2, :), '-o', 'DisplayName', 'x_2');
plot(0:T_final, x(3, :), '-o', 'DisplayName', 'x_3');
title('State Trajectories');
xlabel('Time Steps');
ylabel('States');
legend;
grid on;

% Plot output trajectories
subplot(3, 1, 2);
plot(0:T_final, y(1, :), '-o', 'DisplayName', 'y');
title('Output Trajectory');
xlabel('Time Steps');
ylabel('Output');
legend;
grid on;

% Plot control inputs
subplot(3, 1, 3);
stairs(1:T_final, u, '-o', 'DisplayName', 'u');
title('MPC Control Input');
xlabel('Time Steps');
ylabel('Input');
legend;
grid on;

end

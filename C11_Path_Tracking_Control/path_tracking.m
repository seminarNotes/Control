clc; clear; close all;

%% ==============================
%  Leader의 목표 제어 입력 생성
% ==============================
function control_input_goal = generate_goal_trajectory(t_start, t_count, t_delta)
    Delta = @(t) 0.79 - (sin((2 * pi * (t + 2)) / 3 + 1) + cos((4 * pi * (t + 2)) / 3 + 1)) ./ sqrt(4 * t + 12);

    control_input_goal = zeros(t_count, 2);
    for idx = 1:t_count
        t = t_start + (idx - 1) * t_delta;
        v_goal = 1 + Delta(t);
        w_goal = 1 - Delta(t);
        control_input_goal(idx, :) = [v_goal, w_goal];
    end
end

%% ==============================
%  Error Dynamics (비선형 상태 방정식)
% ==============================
function e = calc_error_state(qL, qF)
    thetaL = qL(3);
    R_T = [cos(thetaL), sin(thetaL), 0;
          -sin(thetaL), cos(thetaL), 0;
           0, 0, 1];

    e = R_T * (qF - qL);
    e(3) = normalize_angle(e(3));
end

function e_dot = error_dynamics(qL, uL, qF, uF)
    vL = uL(1); wL = uL(2);
    vF = uF(1); wF = uF(2);
    
    thetaF = qF(3);
    R_T = [cos(thetaF), sin(thetaF), 0;
          -sin(thetaF), cos(thetaF), 0;
           0, 0, 1];

    e = R_T * (qF - qL);

    e_dot = [vL * cos(e(3)) + wF * e(2) - vF;
             vL * sin(e(3)) - e(1) * wF;
             wL - wF];
end

function dq = mobile_robot_dynamics(q, u)
    theta = q(3);
    S = [cos(theta), 0;
         sin(theta), 0;
         0, 1];
    dq = S * u;
end

function angle = normalize_angle(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end

%% ==============================
%  NMPC 문제 설정 (MATLAB 버전)
% ==============================
function uF_opt = nmpc_path_tracking(qL, qF, uL_traj, k)
    dt = 0.1;
    horizon = 20;  % MPC 예측 지평 (H)

    % 비용 함수 가중치
    Q = diag([10, 10, 3]); 
    R = diag([1, 2]); 
    P = diag([10, 10, 3]); 
    S = diag([10, 15]);

    % 최적화 변수
    U = zeros(2, horizon);

    % 초기 입력 설정
    U(1, :) = uL_traj(k, 1);  
    U(2, :) = uL_traj(k, 2);  

    % 비용 함수 정의
    cost = 0;
    E = zeros(3, horizon + 1);
    E(:, 1) = calc_error_state(qL, qF);

    for i = 1:horizon
        uL_k = uL_traj(i, :);
        e_dot_k = error_dynamics(qL, uL_k, qF, U(:, i));
        E(:, i + 1) = E(:, i) + dt * e_dot_k;

        cost = cost + E(:, i)' * Q * E(:, i) + U(:, i)' * R * U(:, i);
        if i > 1
            delta_u = U(:, i) - U(:, i - 1);
            cost = cost + delta_u' * S * delta_u;
        end
    end
    cost = cost + E(:, end)' * P * E(:, end);

    % 제어 입력 제약조건
    lb = [-2; -2];  % 최소 속도
    ub = [2; 2];  % 최대 속도

    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'none');
    uF_opt = fmincon(@(U) cost, U(:, 1), [], [], [], [], lb, ub, [], options);
end

%% ==============================
%  시뮬레이션 실행
% ==============================
dt = 0.1;
T = 30;
timesteps = T / dt;

% Leader & Follower 초기 상태
qL = [0; 0; 0];
qF = [0; 0; 0];

% Leader 제어 입력 생성
uL_traj = generate_goal_trajectory(0, timesteps, dt);

% 결과 저장
leader_trajectory = zeros(timesteps, 3);
follower_trajectory = zeros(timesteps, 3);
error_trajectory = zeros(timesteps, 3);
mpc_input_trajectory = zeros(timesteps, 2);

for t = 1:timesteps
    uL = uL_traj(t, :);
    uF = nmpc_path_tracking(qL, qF, uL_traj, t);

    qL = qL + dt * mobile_robot_dynamics(qL, uL');
    qF = qF + dt * mobile_robot_dynamics(qF, uF);

    e = calc_error_state(qL, qF);

    leader_trajectory(t, :) = qL';
    follower_trajectory(t, :) = qF';
    error_trajectory(t, :) = e';
    mpc_input_trajectory(t, :) = uF';
end

%% ==============================
%  시각화 (그래프 출력)
% ==============================
figure;
subplot(3, 1, 1);
plot(leader_trajectory(:, 1), leader_trajectory(:, 2), 'b--', 'LineWidth', 1.5);
hold on;
plot(follower_trajectory(:, 1), follower_trajectory(:, 2), 'r-', 'LineWidth', 1.5);
xlabel("X Position");
ylabel("Y Position");
legend("Leader Path", "Follower Path");
title("Path Tracking using Nonlinear MPC");
grid on;

subplot(3, 1, 2);
plot(error_trajectory(:, 1), 'LineWidth', 1.5);
hold on;
plot(error_trajectory(:, 2), 'LineWidth', 1.5);
plot(error_trajectory(:, 3), 'LineWidth', 1.5);
xlabel("Time step");
ylabel("Error");
legend("e_x", "e_y", "e_\theta");
grid on;

subplot(3, 1, 3);
plot(mpc_input_trajectory(:, 1), 'LineWidth', 1.5);
hold on;
plot(mpc_input_trajectory(:, 2), 'LineWidth', 1.5);
xlabel("Time step");
ylabel("MPC input");
legend("v_{mpc}", "w_{mpc}");
grid on;

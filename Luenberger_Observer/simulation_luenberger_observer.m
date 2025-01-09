function simulation_luenberger_observer(A, B, C, D, L)
    % 초기 상태 및 설정
    T_final = 20;               
    T_interval = 0.01;         
    t = 0:T_interval:T_final;   
    sample_size = size(t, 2);  

    % 초기 상태
    X(:, 1) = [1; 0];
    Xhat(:, 1) = [0; 0];
    Ut = [];

    for idx = 1:sample_size - 1
        U = sin(2 * pi * 0.001 * idx) + 0.25 * sin(2 * pi * 0.005 * idx);            
        y(idx) = C * X(:, idx);                  
        yhat(idx) = C * Xhat(:, idx);            
        e(idx) = y(idx) - yhat(idx);             

        % RK4 -> 상태 업데이트
        X(:, idx + 1) = rk4_state_update(A, X(:, idx), B, U, T_interval);
        Xhat(:, idx + 1) = rk4_observer_update(A, Xhat(:, idx), ...
            B, U, L, e(idx), T_interval);

        Ut = [Ut, U]; % 제어 입력 저장
    end

    % 결과 플롯
    figure;
    plot(t, X(1, :), 'DisplayName', 'Actual Position');
    hold on;
    grid on;
    plot(t, Xhat(1, :), 'DisplayName', 'Estimated Position');
    title('Position Estimation');
    legend;
    xlabel('Time (s)');
    ylabel('Position');

    figure;
    plot(t, X(2, :), 'DisplayName', 'Actual Velocity');
    hold on;
    grid on;
    plot(t, Xhat(2, :), 'DisplayName', 'Estimated Velocity');
    title('Velocity Estimation');
    legend;
    xlabel('Time (s)');
    ylabel('Velocity');

    figure;
    plot(t(1:sample_size - 1), Ut);
    title('Control Input');
    xlabel('Time (s)');
    ylabel('Input U');
    grid on;

    figure;
    plot(t(1:sample_size - 1), e);
    title('Estimation Error');
    xlabel('Time (s)');
    ylabel('Error');
    grid on;
end

% 시스템 상태 방정식
function result = state_dynamics(A, x, B, u)
    result = A * x + B * u;
end

% RK4로 상태 업데이트
function result = rk4_state_update(A, X, B, U, T)
    k1 = state_dynamics(A, X, B, U) * T;
    k2 = state_dynamics(A, X + k1 * 0.5, B, U) * T;
    k3 = state_dynamics(A, X + k2 * 0.5, B, U) * T;
    k4 = state_dynamics(A, X + k3, B, U) * T;
    result = X + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
end

% 옵저버 상태 방정식
function result = observer_dynamics(A, x, B, u, L, e)
    result = A * x + B * u + L * e;
end

% % RK4로 옵저버 상태 업데이트
function result = rk4_observer_update(A, X, B, U, L, e, T)
    k1 = observer_dynamics(A, X, B, U, L, e) * T;
    k2 = observer_dynamics(A, X + k1 * 0.5, B, U, L, e) * T;
    k3 = observer_dynamics(A, X + k2 * 0.5, B, U, L, e) * T;
    k4 = observer_dynamics(A, X + k3, B, U, L, e) * T;
    result = X + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
end

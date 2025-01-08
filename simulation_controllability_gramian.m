
function simulation_controllability_gramian(A, B, C, D, x_initial, x_desired)
    % 초기 상태 및 목표 상태 설정
    T_final = 80;
    T_interval = 0.1;
    t = 0:T_interval:T_final; 
    sample_size = size(t, 2);

    X(:, 1) = x_initial;

    % 제어 가능 Grammian 계산
    Integrand = @(tau) expm(A * tau) * B * B' * expm(A' * tau); 
    W = @(t) integral(Integrand, 0, t, 'ArrayValued', true);
    W_final = W(T_final);

    % 상태와 입력 계산
    for i = 1:sample_size - 1
        U(i) = B' * expm(A' * (T_final - t(i))) * inv(W_final) * x_desired;
        X(:, i + 1) = runge_kutta(A, X(:, i), B, U(i), T_interval);
    end

    % 입력과 상태 플롯
    figure(1);
    plot(t(1:end - 1), U);
    grid on;
    title('Input');
    xlabel('Time step');
    ylabel('u(t)');

    figure(2);
    plot(t, X);
    grid on;
    title('State');
    xlabel('Time step');
    ylabel('State values');
    legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
    
    %% 
    % 목표 상태와 마지막 상태의 차이 계산 및 출력
    diff = x_desired - X(:, end);

    fprintf('x_desired = ');
    fprintf('\n');
    disp(x_desired);

    fprintf(' x_reached = ');
    fprintf('\n');
    disp(X(:, end));
    
    fprintf('x_desired - x_reached = ');
    fprintf('\n');
    disp(diff);

end

% Runge-Kutta 4
function result = runge_kutta(A, X, B, U, T)
    k1 = state_equation(A, X, B, U) * T;
    k2 = state_equation(A, X + k1 * 0.5, B, U) * T;
    k3 = state_equation(A, X + k2 * 0.5, B, U) * T;
    k4 = state_equation(A, X + k3, B, U) * T;
    result = X + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
end

% 시스템 동역학 함수
function result = state_equation(A, x, B, u)
    result = A * x + B * u;
end

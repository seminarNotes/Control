order = 2;

if order == 3
    A = [0  1  0;
         0  0  1;
        -2 -3 -4];

    B = [0; 0; 1];
    C = [1 0 0];
    D = 0;

    x_initial = [1; 0; 0];
    x_desired = [3; 2; 0.4];

elseif order == 2
    A = [0  1;
        -2 -3];
    B = [0; 1];
    C = [1 0];
    D = 0;

    x_initial = [1; 0];
    x_desired = [2; 1];

else
    exit;
end

simulation_controllability_gramian(A, B, C, D, x_initial, x_desired);

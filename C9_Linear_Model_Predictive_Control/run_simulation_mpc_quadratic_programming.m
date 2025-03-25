% run_simulation_mpc_quadratic_programming.m

A = [2.4936 -1.2130 0.4176;
     2       0       0;
     0       1       0];
B = [0.25; 0; 0];
C = [0.0684 0.1288 0.0313];
D = 0;

simulation_mpc_quadratic_programming(A, B, C, D)
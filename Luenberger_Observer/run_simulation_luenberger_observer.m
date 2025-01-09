% run_simulation_luenberger_observer.m
clc;
clear;

% 시스템 행렬 정의
A = [0 1; 0 -1];
B = [0; 1];
C = [1 0];
D = 0;
L = [2*sqrt(3) - 1; 7 - 2*sqrt(3)]; 

simulation_luenberger_observer(A, B, C, D, L);

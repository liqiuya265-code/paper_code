% 测试权重分配功能
clc; clear;

% 测试参数
kappa = 0.01;
lambda = 0.1;
d_crit = 200;

% 创建障碍物对象
d_safe = 300;
kappa1 = 1;
kappa2 = 1;
obs = obstacles(d_safe, kappa1, kappa2);
obs.add_spherical_obstacle([1000, 0, 0], 100);

% 测试位置
p_i = [900, 0, 0];  % 接近障碍物

% 测试环境安全因子
obstacle_detected = true;
phi_i = environmental_safety_factor(obs, p_i, obstacle_detected, kappa, d_crit);
fprintf('环境安全因子 φ_i = %.4f\n', phi_i);

% 测试信息可信因子（模拟观测器数据）
M = 4;
z_observer = zeros(M, M*5);
x_true = zeros(1, M*5);
a_now = [1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
i = 1;
psi_i = information_credibility_factor(z_observer, x_true, a_now, i, lambda);
fprintf('信息可信因子 ψ_i = %.4f\n', psi_i);

% 测试权重计算
[omega_1i, omega_2i] = compute_weights(phi_i, psi_i);
fprintf('权重 ω_{1i} = %.4f, ω_{2i} = %.4f\n', omega_1i, omega_2i);

% 测试加权控制分配
a_N = [0; 0; 0];  % 名义控制
a_S = [1; 0; 0];  % 避障控制
a_T = [0; 1; 0];  % 协同控制
F_i = 1;  % 避障优先

a_i = weighted_control_allocation(a_N, a_S, a_T, F_i, omega_1i, omega_2i);
fprintf('最终控制 a_i = [%.4f, %.4f, %.4f]\n', a_i(1), a_i(2), a_i(3));

disp('权重分配功能测试完成！');
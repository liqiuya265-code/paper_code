% 测试观测器权重分配功能
clc; clear;

% 测试参数
M = 4;
kappa_env = 0.01;
lambda_info = 0.1;
d_crit = 200;

% 创建障碍物对象
d_safe = 300;
kappa1 = 1;
kappa2 = 1;
obs = obstacles(d_safe, kappa1, kappa2);
obs.add_spherical_obstacle([1000, 0, 0], 100);

% 创建模拟的观测器状态
z_observer = zeros(M, M*5);
x_true = zeros(1, M*5);
a_now = [1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];

% 设置一些观测误差
for i = 1:M
    for j = 1:M
        if i == j
            z_observer(i, 5*(j-1)+1:5*j) = [15000-500*i, 0, 0, 0, 0];  % 距离有偏差
            x_true(5*(j-1)+1:5*j) = [15000-500*i, 0, 0, 0, 0];
        else
            z_observer(i, 5*(j-1)+1:5*j) = [15000-500*j + 100, 0.1, 0.1, 0.1, 0.1];  % 有观测误差
            x_true(5*(j-1)+1:5*j) = [15000-500*j, 0, 0, 0, 0];
        end
    end
end

% 测试观测器权重分配
for j = 1:M
    % 计算导弹当前位置（基于观测状态）
    r_obs_j = z_observer(j, 5*(j-1)+1);
    theta_L_obs_j = z_observer(j, 5*(j-1)+2);
    psi_L_obs_j = z_observer(j, 5*(j-1)+3);
    p_i_obs = [-r_obs_j*cos(theta_L_obs_j)*cos(psi_L_obs_j), ...
        -r_obs_j*cos(theta_L_obs_j)*sin(psi_L_obs_j), ...
        -r_obs_j*sin(theta_L_obs_j)];

    % 计算环境安全因子 φ_i
    obstacle_detected_obs = false;
    phi_i_obs = environmental_safety_factor(obs, p_i_obs, obstacle_detected_obs, kappa_env, d_crit);

    % 计算信息可信因子 ψ_i
    psi_i_obs = information_credibility_factor(z_observer, x_true, a_now, j, lambda_info);

    % 计算权重 ω_{1i} 和 ω_{2i}
    [omega_1i_obs, omega_2i_obs] = compute_weights(phi_i_obs, psi_i_obs);

    fprintf('导弹 %d - 观测器权重: ω_{1i}=%.4f, ω_{2i}=%.4f, φ_i=%.4f, ψ_i=%.4f\n', ...
        j, omega_1i_obs, omega_2i_obs, phi_i_obs, psi_i_obs);
end

disp('观测器权重分配测试完成！');
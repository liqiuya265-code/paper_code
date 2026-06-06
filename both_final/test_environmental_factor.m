% 测试新的环境安全因子计算
clc; clear;

% 创建障碍物对象
d_safe = 300;
kappa1 = 1;
kappa2 = 1;
obs = obstacles(d_safe, kappa1, kappa2);

% 添加测试障碍物
obs.add_spherical_obstacle([1000, 0, 0], 100);  % 中心在(1000,0,0)，半径100

% 新的测试参数
omega = 1.5;  % 障碍安全系数 ω > 1
n = 2;        % 余弦函数指数

fprintf('测试参数: ω = %.1f, n = %d, 障碍物半径 R = 100m\n', omega, n);
fprintf('安全边界 = R + d_safe = %dm\n\n', 100 + d_safe);

% 测试点1：在 ωR = 1.5*100 = 150m 之外（安全）
p_i1 = [950, 0, 0];  % 距离障碍物中心950m
r_oi1 = 950 - (100 + 300);  % 到安全边界的距离 = 950 - 400 = 550m
obstacle_detected1 = true;
phi_i1 = environmental_safety_factor(obs, p_i1, obstacle_detected1, omega, n);
fprintf('测试点1（r_oi = %dm > ωR = %dm）: phi_i = %.4f (期望 ≈ 1.0)\n', r_oi1, round(omega*100), phi_i1);

% 测试点2：在 ωR 附近
p_i2 = [525, 0, 0];  % 距离障碍物中心525m
r_oi2 = 525 - (100 + 300);  % 到安全边界的距离 = 525 - 400 = 125m
obstacle_detected2 = true;
phi_i2 = environmental_safety_factor(obs, p_i2, obstacle_detected2, omega, n);
fprintf('测试点2（r_oi = %dm, ωR = %dm）: phi_i = %.4f\n', r_oi2, round(omega*100), phi_i2);

% 测试点3：在 ωR 内（需要避障）
p_i3 = [475, 0, 0];  % 距离障碍物中心475m
r_oi3 = 475 - (100 + 300);  % 到安全边界的距离 = 475 - 400 = 75m
obstacle_detected3 = true;
phi_i3 = environmental_safety_factor(obs, p_i3, obstacle_detected3, omega, n);
fprintf('测试点3（r_oi = %dm < ωR = %dm）: phi_i = %.4f (期望 < 1.0)\n', r_oi3, round(omega*100), phi_i3);

% 测试点4：在安全边界内（严重警告区）
p_i4 = [350, 0, 0];  % 距离障碍物中心350m
r_oi4 = 350 - (100 + 300);  % 到安全边界的距离 = 350 - 400 = -50m（侵入50m）
obstacle_detected4 = true;
phi_i4 = environmental_safety_factor(obs, p_i4, obstacle_detected4, omega, n);
fprintf('测试点4（r_oi = %dm，侵入安全区）: phi_i = %.4f (期望很小)\n', r_oi4, phi_i4);

% 测试点5：无障碍物检测
p_i5 = [2000, 0, 0];  % 远离障碍物
obstacle_detected5 = false;
phi_i5 = environmental_safety_factor(obs, p_i5, obstacle_detected5, omega, n);
fprintf('测试点5（无障碍物）: phi_i = %.4f (期望 = 1.0)\n', phi_i5);

disp('新的环境安全因子测试完成！');
% 简单测试障碍物函数
clc; clear;

% 初始化障碍物系统
d_safe = 200;
obs = obstacles(d_safe, 1.0, 1.0);

% 添加一个球形障碍物
obs.add_spherical_obstacle([8000, 2000, 3000], 500);

% 测试位置
p_i = [8100, 2100, 3100]';  % 应该在危险区域内

% 手动计算
p_o = [8000, 2000, 3000]';
r_o = p_i - p_o;
r_o_norm_sq = r_o' * r_o;
h_expected = r_o_norm_sq - d_safe^2;

fprintf('p_i: [%.1f, %.1f, %.1f]\n', p_i(1), p_i(2), p_i(3));
fprintf('p_o: [%.1f, %.1f, %.1f]\n', p_o(1), p_o(2), p_o(3));
fprintf('r_o: [%.1f, %.1f, %.1f]\n', r_o(1), r_o(2), r_o(3));
fprintf('r_o_norm_sq: %.1f\n', r_o_norm_sq);
fprintf('d_safe^2: %.1f\n', d_safe^2);
fprintf('h_expected: %.1f\n', h_expected);

% 直接计算
r_o_direct = p_i - p_o;
r_o_norm_sq_direct = r_o_direct' * r_o_direct;
h_direct = r_o_norm_sq_direct - d_safe^2;
fprintf('h_direct: %.1f\n', h_direct);

% 使用函数计算
h_actual = obs.spherical_barrier_function(p_i, 1);
fprintf('h_actual: %.1f\n', h_actual);

disp('测试完成');
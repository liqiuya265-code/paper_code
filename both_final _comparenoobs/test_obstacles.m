% 测试障碍物功能
clc; clear;

% 初始化障碍物系统
d_safe = 200;  % 安全距离200米
kappa1 = 1.0;  % CBF参数 κ₁
kappa2 = 1.0;  % CBF参数 κ₂
obs = obstacles(d_safe, kappa1, kappa2);

% 添加球形障碍物示例
obs.add_spherical_obstacle([8000, 2000, 3000], 500);  % 球心位置和半径
obs.add_spherical_obstacle([5000, -3000, 1000], 400);

% 添加圆柱形障碍物示例 (垂直轴线)
obs.add_cylindrical_obstacle([10000, 0, 0], 300, [0, 0, 1]);  % 轴线上一点、半径、轴线方向
obs.add_cylindrical_obstacle([3000, 4000, 0], 250, [0, 0, 1]);

% 测试障碍物检测 - 远离障碍物的情况
test_position1 = [8500, 2500, 3500]';  % 靠近第一个球形障碍物（列向量）
test_velocity1 = [300, 0, 0]';  % 导弹速度（列向量）
nominal_acceleration1 = [0, 0, 0]';  % 名义控制输入（列向量）
[avoidance_force1, obstacle_detected1] = obs.compute_obstacle_avoidance(test_position1, test_velocity1, nominal_acceleration1);

fprintf('测试位置1 (远离): [%.1f, %.1f, %.1f]\n', test_position1);
fprintf('检测到障碍物1: %d\n', obstacle_detected1);
fprintf('避免力1: [%.2f, %.2f, %.2f]\n\n', avoidance_force1);

% 测试障碍物检测 - 接近障碍物的情况
test_position2 = [8050, 2050, 3050]';  % 非常接近第一个球形障碍物 (距离约50米)
test_velocity2 = [300, 0, 0]';  % 导弹速度
nominal_acceleration2 = [0, 0, 0]';  % 名义控制输入
[avoidance_force2, obstacle_detected2] = obs.compute_obstacle_avoidance(test_position2, test_velocity2, nominal_acceleration2);

fprintf('测试位置2 (接近): [%.1f, %.1f, %.1f]\n', test_position2);
fprintf('检测到障碍物2: %d\n', obstacle_detected2);
fprintf('避免力2: [%.2f, %.2f, %.2f]\n\n', avoidance_force2);

% 测试圆柱形障碍物
test_position3 = [10100, 100, 0]';  % 接近第一个圆柱形障碍物
test_velocity3 = [300, 0, 0]';  % 导弹速度
nominal_acceleration3 = [0, 0, 0]';  % 名义控制输入
[avoidance_force3, obstacle_detected3] = obs.compute_obstacle_avoidance(test_position3, test_velocity3, nominal_acceleration3);

fprintf('测试位置3 (圆柱形): [%.1f, %.1f, %.1f]\n', test_position3);
fprintf('检测到障碍物3: %d\n', obstacle_detected3);
fprintf('避免力3: [%.2f, %.2f, %.2f]\n', avoidance_force3);

% 测试绘制
figure(1);
clf;
obs.plot_obstacles();
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('障碍物可视化测试');
grid on;
axis equal;
view(3);

disp('障碍物测试完成');

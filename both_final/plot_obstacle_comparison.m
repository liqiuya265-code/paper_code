% 避障对比脚本：有避障（实线）vs 无避障（虚线）
% 在同一张图上对比 main.m 和 main_compare2.m 的仿真结果
clc;clear
%% 公共参数设置
tf=100; dt=0.1; t=0:dt:tf;
Vm=[300,300,300,300]; N=4; M=4;
sigma_max=5*pi/180;
alpha=5; beta=5; p=0.8; q=1.2; m=1.5; miu=0.3; v=0.7; n=3; m1=0; varpi=2;
lambda_info = 0.0008;

% 通信拓扑
a_base=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
a_now=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];

% DoS 参数
zeta_ij=1*ones(M); mu_ij=0.52*ones(M);
kappa_ij=2*ones(M); nu_ij=5*ones(M);
attack_prob=0.1*ones(M);
dos_downtime=zeros(M); dos_active=zeros(M);
dos_event_count=zeros(M); dos_last_event_time=zeros(M);
t0=0; rng(1);

% 加载DoS场景
load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');

% 初始状态
x0=[12500,-45*pi/180,45*pi/180,30*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-45*pi/180,45*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,30*pi/180,-30*pi/180];

T_safe = 5; T = 10;

% 障碍物参数（d_safe 与 main.m 保持一致）
d_safe = 50; kappa1 = 1; kappa2 =1;
omega_env_i =2.5*ones(1, M); n_env = 2;

%% ===== 仿真 1：有避障 =====
disp('Running simulation WITH obstacle avoidance...');
rng(1);  % 重置随机种子，保证初始观测器偏差一致
use_obstacle_avoidance = true;
[X_obs, Y_obs, Z_obs, tgo_obs, sigma_obs, Ay_obs, Az_obs, ...
    x_state_obs, weights_log_obs, z_observer_log_obs, len_obs_sim] = ...
    run_single_sim(t, dt, Vm, N, M, sigma_max, alpha, beta, p, q, m, miu, v, n, ...
        a_base, a_now, a_log, dos_downtime_log, dos_active_log, dos_event_count_log, ...
        x0, T_safe, T, lambda_info, d_safe, kappa1, kappa2, omega_env_i, n_env, ...
        use_obstacle_avoidance, m1, 'both');
fprintf('  Obstacle detected at %d time steps.\n', sum(any(weights_log_obs(:,:,1) > 0, 2)));

%% ===== 仿真 2：无避障（仅观测器处理DoS）=====
disp('Running simulation WITHOUT obstacle avoidance...');
rng(1);  % 重置随机种子，保证初始观测器偏差一致
use_obstacle_avoidance = false;
[X_no, Y_no, Z_no, tgo_no, sigma_no, Ay_no, Az_no, ...
    x_state_no, weights_log_no, z_observer_log_no, len_no_sim] = ...
    run_single_sim(t, dt, Vm, N, M, sigma_max, alpha, beta, p, q, m, miu, v, n, ...
        a_base, a_now, a_log, dos_downtime_log, dos_active_log, dos_event_count_log, ...
        x0, T_safe, T, lambda_info, d_safe, kappa1, kappa2, omega_env_i, n_env, ...
        use_obstacle_avoidance, m1, 'both');

disp('Both simulations complete. Generating comparison figures...');

% 保存控制量数据用于控制量对比
save('control_effort_obs.mat', 'Ay_obs', 'Az_obs', 't');
save('control_effort_no_obs.mat', 'Ay_no', 'Az_no', 't');
fprintf('Control effort data saved: control_effort_obs.mat, control_effort_no_obs.mat\n');

%% ===== 对比绘图 =====
% 统一时间长度
len_obs = len_obs_sim;
len_no = len_no_sim;

% 障碍物对象（用于绘图）
obs_plot = obstacles(d_safe, kappa1, kappa2);
obs_plot.add_spherical_obstacle([-500, -3500, 4000], 500);  % 阻挡 M1 路径
obs_plot.add_cylindrical_obstacle([-5000, -1800, 0], 500, [0, 0, 1]);  % 阻挡 M2 路径（垂直圆柱）
obs_plot.add_spherical_obstacle([-2000, -500, 4500], 500);  % 阻挡 M4 路径
obs_plot.add_cylindrical_obstacle([-2000, -2800, 0], 500, [0, 0, 1]);  % 阻挡 M4 路径
colors = lines(M);

% ---- 图1：3D轨迹对比 ----
figure(1)
set(gcf, 'Position', [100, 100, 800, 600]);
hold on;
for j = 1:M
    plot3(X_obs(1:len_obs, j), Y_obs(1:len_obs, j), Z_obs(1:len_obs, j), ...
        '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot3(X_no(1:len_no, j), Y_no(1:len_no, j), Z_no(1:len_no, j), ...
        '--', 'LineWidth', 2, 'Color', colors(j,:));
end
% 绘制障碍物
obs_plot.plot_obstacles();
plot3(0,0,0,'ko','MarkerSize',10,'LineWidth',2);
text(0,0,0,' Target','FontSize',11);
zlim([0 8000]);
set(gca, 'XDir', 'reverse', 'YDir', 'reverse');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; view(135, 20);
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% ---- 图2：tgo 与 距离 r 对比（2×1）----
figure(2)
set(gcf, 'Position', [100, 100, 800, 600]);

subplot(2,1,1); hold on;
for j = 1:M
    plot(t(1:len_obs), tgo_obs(1:len_obs, j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:len_no), tgo_no(1:len_no, j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
ylabel('t_{go} (s)'); xlabel('t(s)');grid on;

subplot(2,1,2); hold on;
r_obs_all = x_state_obs(1:len_obs, 1:5:20);
r_no_all = x_state_no(1:len_no, 1:5:20);
for j = 1:M
    plot(t(1:len_obs), r_obs_all(:, j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:len_no), r_no_all(:, j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('R (m)'); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% ---- 图3：加速度 Ay, Az 对比 ----
figure(3)
set(gcf, 'Position', [100, 100, 800, 600]);
subplot(2,1,1); hold on;
for j = 1:M
    plot(t(1:len_obs), Ay_obs(1:len_obs, j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:len_no), Ay_no(1:len_no, j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('A_z (m/s^2)');
ylabel('A_y (m/s^2)'); grid on;

subplot(2,1,2); hold on;
for j = 1:M
    plot(t(1:len_obs), Az_obs(1:len_obs, j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:len_no), Az_no(1:len_no, j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('A_z (m/s^2)'); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% ---- 图4：前置角 sigma 对比 ----
figure(4)
set(gcf, 'Position', [100, 100, 800, 600]);
hold on;
for j = 1:M
    plot(t(1:len_obs), rad2deg(sigma_obs(1:len_obs, j)), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:len_no), rad2deg(sigma_no(1:len_no, j)), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('\sigma (deg)'); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);



% ---- 图5：F_i 模式选择因子（分导弹显示避障触发时段）----
figure(5)
set(gcf, 'Position', [350, 50, 800, 600]);
for midx = 1:M
    subplot(M, 1, midx); hold on;
    F_vals = squeeze(weights_log_obs(1:len_obs, midx, 1));
    stairs(t(1:len_obs), F_vals, '-', 'LineWidth', 1.8, 'Color', colors(midx,:));
    % 标记避障激活区域（半透明底色）
    active_mask = F_vals > 0;
    if any(active_mask)
        t_active = t(1:len_obs);
        y_region = [0 1.05];
        % 找到激活段并填充
        transitions = diff([0; active_mask; 0]);
        start_idx = find(transitions == 1);
        end_idx = find(transitions == -1) - 1;
        for seg = 1:length(start_idx)
            if start_idx(seg) > 1
                t_start = t(start_idx(seg)-1);
            else
                t_start = t(1);
            end
            if end_idx(seg) < len_obs
                t_end = t(end_idx(seg)+1);
            else
                t_end = t(len_obs);
            end
            fill([t_start t_end t_end t_start], [y_region(1) y_region(1) y_region(2) y_region(2)], ...
                colors(midx,:), 'FaceAlpha', 0.12, 'EdgeColor', 'none');
        end
    end
    ylim([-0.1, 1.1]); yticks([0 1]); yticklabels({'Coop.', 'Avoid'});
    xlabel('t (s)'); ylabel('F_i');
    grid on; hold off;
end
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);
sgtitle('F_i Mode Selector (1=Obstacle Avoidance Active, 0=Cooperation Only)', ...
    'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

% ---- 图6：权重因子对比 φ_i, ψ_i, ω_{2i}=φ_i·ψ_i（3行×1列）----
figure(6)
set(gcf, 'Position', [100, 80, 800, 600]);

% --- 子图1：φ_i（环境安全因子）---
subplot(3,1,1); hold on;
for midx = 1:M
    phi_obs = squeeze(weights_log_obs(1:len_obs, midx, 3));
    phi_no  = squeeze(weights_log_no(1:len_no, midx, 3));
    plot(t(1:len_obs), phi_obs, '-', 'LineWidth', 2, 'Color', colors(midx,:));
    plot(t(1:len_no),  phi_no,  '--', 'LineWidth', 2, 'Color', colors(midx,:));
end
ylim([-0.05, 1.05]); ylabel('\eta_{2,i}'); xlabel('t(s)');grid on;

% --- 子图2：ψ_i（信息可信因子）---
subplot(3,1,2); hold on;
for midx = 1:M
    psi_obs = squeeze(weights_log_obs(1:len_obs, midx, 4));
    psi_no  = squeeze(weights_log_no(1:len_no, midx, 4));
    plot(t(1:len_obs), psi_obs, '-', 'LineWidth', 2, 'Color', colors(midx,:));
    plot(t(1:len_no),  psi_no,  '--', 'LineWidth', 2, 'Color', colors(midx,:));
end
ylim([-0.05, 1.05]); ylabel('\eta_{1,i}'); xlabel('t(s)');grid on;


% --- 子图3：ω_{2i} = φ_i·ψ_i（复合权重）---
subplot(3,1,3); hold on;
for midx = 1:M
    omega_obs = squeeze(weights_log_obs(1:len_obs, midx, 2));
    omega_no  = squeeze(weights_log_no(1:len_no, midx, 2));
    plot(t(1:len_obs), omega_obs, '-', 'LineWidth', 2, 'Color', colors(midx,:));
    plot(t(1:len_no),  omega_no,  '--', 'LineWidth', 2, 'Color', colors(midx,:));
end
ylim([-0.05, 1.05]); ylabel('\eta_{1,i} \times \eta_{2,i}'); grid on;
xlabel('t(s)');

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% ---- 图7：统一图例（2×4，完全居中）----
figure(7)
set(gcf, 'Position', [200, 420, 680, 170], 'Color', 'w');
ax = axes('Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
hold on;

% 图例背景框
rectangle('Position', [0.03, 0.04, 0.94, 0.3], 'FaceColor', 'w', ...
     'LineWidth', 0.8);

n_col = M;
col_w = 0.88 / n_col;
start_x = 0.08;

% 右侧行标注
text(0.1, 0.255, 'OA:',   'FontSize', 12, 'FontName', 'Times New Roman', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
text(0.1, 0.105, 'noOA:', 'FontSize', 12, 'FontName', 'Times New Roman', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');

for midx = 1:n_col
    cx = start_x + (midx - 0.5) * col_w;   % 列中心

    % 上行：实线 = OA
    line([cx-0.07, cx], [0.255, 0.255], 'LineWidth', 2.5, 'Color', colors(midx,:));
    text(cx+0.02, 0.255, sprintf('Missile %d', midx), ...
        'FontSize', 12, 'FontName', 'Times New Roman', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');

    % 下行：虚线 = noOA
    line([cx-0.07, cx], [0.105, 0.105], 'LineWidth', 2.5, ...
        'LineStyle', '--', 'Color', colors(midx,:));
    text(cx+0.02, 0.105, sprintf('Missile %d', midx), ...
        'FontSize', 12, 'FontName', 'Times New Roman', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
end
set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% ---- 图8：观测器状态估计误差范数对比（OA vs noOA，模仿 main_base Fig5）----
% 对每个观测导弹 i，显示其对其他导弹的 5 维状态误差 2-范数
len_obs_err = min([len_obs, len_no, size(x_state_obs,1), size(z_observer_log_obs,1), ...
                   size(x_state_no,1), size(z_observer_log_no,1)]);
t_plot_obs = t(1:len_obs_err);

figure(8)
set(gcf, 'Position', [100, 50, 800, 600]);
roman_labels = {'i', 'ii', 'iii', 'iv'};
for i_obs = 1:M
    subplot(4,1,i_obs)
    hold on;
    for j_target = 1:M
        if i_obs ~= j_target
            err_obs_vec = zeros(len_obs_err, 1);
            for k = 1:len_obs_err
                e_o = zeros(5, 1);
                for state_idx = 1:5
                    real_state_idx = 5*(j_target-1) + state_idx;
                    obs_idx = 20*(i_obs-1) + 5*(j_target-1) + state_idx;
                    e_o(state_idx) = z_observer_log_obs(k, obs_idx) - x_state_obs(k, real_state_idx);
                end
                err_obs_vec(k) = norm(e_o);
            end
            plot(t_plot_obs, err_obs_vec, '-', 'LineWidth', 2, 'Color', colors(j_target,:), ...
                'DisplayName', ['to Missile ', num2str(j_target)]);
        end
    end
    if i_obs == M
        xlabel("t(s)")
    end
    ylabel("||e_{state}||_2")
    text(0.5, 0.8, ['(', roman_labels{i_obs}, ') Missile ', num2str(i_obs), ' State Error Norm'], ...
        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
        'FontSize', 12, 'FontName', 'Times New Roman', 'Clipping', 'off');
    legend('Location', 'best')
    grid on;
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);
set(findall(gcf, 'Type', 'Legend'), 'FontSize', 12);


all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);
% 图5 legend 单独设为 12pt
set(findall(gcf, 'Type', 'Legend'), 'FontSize', 12);




%% ===== 导出高分辨率 PDF =====
pdf_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~exist(pdf_dir, 'dir'), mkdir(pdf_dir); end
fig_names = {'trajectory', 'tgo_range', 'acceleration', 'sigma', 'Fi', 'weights', 'legend', ...
             'observer_error_norm', 'observer_range_error'};
for fnum = 1:8
    fname = fullfile(pdf_dir, sprintf('fig%d_%s.pdf', fnum, fig_names{fnum}));
    figure(fnum);
    set(gcf, 'PaperPositionMode', 'auto');
    exportgraphics(gcf, fname, 'Resolution', 600, 'ContentType', 'vector', 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', fname);
end
disp('All PDF figures exported.');

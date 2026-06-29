clc;clear;

% ========== 公共参数 ==========
tf = 100;
dt = 0.1;
t = 0:dt:tf;
Vm = [300,300,300,300];
N = 4;
M = 4;

%% ========== 运行两次仿真 ==========
% 考虑弹性因子 (omega_2i = psi_i * phi_i)
rng(1);
res = run_simulation(true, tf, dt, t, Vm, N, M);

% 不考虑弹性因子 (omega_2i = 1)
rng(1);
nores = run_simulation(false, tf, dt, t, Vm, N, M);

% 保存控制量数据用于对比
Ay_res = res.Ay; Az_res = res.Az; %#ok<NASGU>
Ay_nores = nores.Ay; Az_nores = nores.Az; %#ok<NASGU>
save('control_effort_base.mat', 'Ay_res', 'Az_res', 't');
save('control_effort_no_res.mat', 'Ay_nores', 'Az_nores', 't');

%% ========== 对比绘图：实线=有弹性因子，虚线=无弹性因子 ==========

% --- 图1: 3D轨迹对比 ---
figure(1); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
hold on;
colors = lines(M);
for j = 1:M
    plot3(res.X(:,j), res.Y(:,j), res.Z(:,j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot3(nores.X(:,j), nores.Y(:,j), nores.Z(:,j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
set(gca, 'XDir', 'reverse');
set(gca, 'YDir', 'reverse');
plot3(0,0,0, 'Marker', 'o', 'LineWidth', 2);
text(0,0,0, 'Target');
xlabel("X(m)"); ylabel("Y(m)"); zlabel("Z(m)");
grid on;
view(3);
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% --- 图2: tgo 与 Range 对比 ---
figure(2); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
t_end = max(res.t_end, nores.t_end);
n_res = res.actual_steps;
n_nores = nores.actual_steps;

subplot(2,1,1); hold on;
for j = 1:M
    plot(t(1:n_res), res.tgo(1:n_res,j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:n_nores), nores.tgo(1:n_nores,j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
ylabel("t_{go}(s)"); grid on; xlim([0, t_end]);
set(gca, 'XTickLabel', []);

subplot(2,1,2); hold on;
n_st_res = size(res.x_state, 1);
n_st_nores = size(nores.x_state, 1);
for j = 1:M
    plot(t(1:n_st_res), res.x_state(:, 5*(j-1)+1), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:n_st_nores), nores.x_state(:, 5*(j-1)+1), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel("t(s)"); ylabel("R(m)"); grid on; xlim([0, t_end]);

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% --- 图3: 多通道DoS攻击示意图（仅一份，供参考） ---
link_list_all = [1,2; 2,3; 3,4; 4,1];
num_links_all = size(link_list_all, 1);
link_labels_all = cell(num_links_all, 1);
for k = 1:num_links_all
    link_labels_all{k} = sprintf('(%d,%d)', link_list_all(k,1), link_list_all(k,2));
end
load('dos_scenario.mat', 'a_log');
len_a = min(length(a_log), length(t));
attack_matrix = zeros(len_a, num_links_all);
for k = 1:num_links_all
    r = link_list_all(k, 1);
    c = link_list_all(k, 2);
    link_stat = squeeze(a_log(1:len_a, r, c));
    attack_matrix(:, k) = (link_stat == 0);
end
t_plot_mc = t(1:len_a);

figure(3); clf;
set(gcf, 'Position', [100, 100, 750, 500]);
gap_val = 0.02;
margin_bottom = 0.08;
margin_top = 0.06;
avail_h = 1 - margin_bottom - margin_top - (num_links_all-1)*gap_val;
row_h = avail_h / num_links_all;
colors_dos = lines(M);
for k = 1:num_links_all
    y_bottom = margin_bottom + (num_links_all-k)*(row_h + gap_val);
    subplot('Position', [0.12, y_bottom, 0.85, row_h]);
    hold on;
    stairs(t_plot_mc, attack_matrix(:, k), '-', 'Color', colors_dos(k,:), 'LineWidth', 1.5);
    ylim([-0.15, 1.15]);
    yticks([0, 1]); yticklabels({'Safe', 'DoS'});
    xlim([0, t_plot_mc(end)]);
    ylabel(link_labels_all{k}, 'FontSize', 12, 'FontName', 'Times New Roman', ...
        'Rotation', 0, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    if k < num_links_all
        set(gca, 'XTickLabel', []);
    end
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 0.5);
    grid on;
end
xlabel('t(s)', 'FontSize', 12, 'FontName', 'Times New Roman');
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% --- 图4: Ay 与 Az 对比 ---
figure(4); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
subplot(2,1,1); hold on;
for j = 1:M
    plot(t(1:n_res), res.Ay(1:n_res,j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:n_nores), nores.Ay(1:n_nores,j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
ylabel("Ay(m/s^2)"); grid on; xlim([0, t_end]);

subplot(2,1,2); hold on;
for j = 1:M
    plot(t(1:n_res), res.Az(1:n_res,j), '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:n_nores), nores.Az(1:n_nores,j), '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel("t(s)"); ylabel("Az(m/s^2)"); grid on; xlim([0, t_end]);

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% --- 图5: 观测器误差对比 ---
len_obs_res = min(size(res.x_state,1), size(res.z_observer_log,1));
len_obs_nores = min(size(nores.x_state,1), size(nores.z_observer_log,1));
roman_labels = {'i', 'ii', 'iii', 'iv'};

figure(5); clf;
set(gcf, 'Position', [50, 50, 800, 900]);
for i_obs = 1:M
    subplot(4,1,i_obs); hold on;
    for j_target = 1:M
        if i_obs ~= j_target
            % 有弹性因子
            error_norm_res = zeros(len_obs_res, 1);
            for k = 1:len_obs_res
                e_state = zeros(5, 1);
                for state_idx = 1:5
                    real_idx = 5*(j_target-1) + state_idx;
                    obs_idx = 20*(i_obs-1) + 5*(j_target-1) + state_idx;
                    e_state(state_idx) = res.z_observer_log(k, obs_idx) - res.x_state(k, real_idx);
                end
                error_norm_res(k) = norm(e_state);
            end
            plot(t(1:len_obs_res), error_norm_res, '-', 'LineWidth', 2, 'Color', colors(j_target,:));

            % 无弹性因子
            error_norm_nores = zeros(len_obs_nores, 1);
            for k = 1:len_obs_nores
                e_state = zeros(5, 1);
                for state_idx = 1:5
                    real_idx = 5*(j_target-1) + state_idx;
                    obs_idx = 20*(i_obs-1) + 5*(j_target-1) + state_idx;
                    e_state(state_idx) = nores.z_observer_log(k, obs_idx) - nores.x_state(k, real_idx);
                end
                error_norm_nores(k) = norm(e_state);
            end
            plot(t(1:len_obs_nores), error_norm_nores, '--', 'LineWidth', 2, 'Color', colors(j_target,:));
        end
    end
    ylabel("||e_{state}||_2");
    text(0.5, 0.8, ['(', roman_labels{i_obs}, ') Missile ', num2str(i_obs), ' State Error Norm'], ...
        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
        'FontSize', 12, 'FontName', 'Times New Roman', 'Clipping', 'off');
    if i_obs == M
        xlabel("t(s)");
    end
    grid on;
    xline(10, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xlim([0, t_end]);
end
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% --- 图6: 前置角 sigma 对比 ---
figure(6); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
hold on;
for j = 1:M
    sigma_deg_res = rad2deg(res.sigma(1:n_res, j));
    sigma_deg_nores = rad2deg(nores.sigma(1:n_nores, j));
    plot(t(1:n_res), sigma_deg_res, '-', 'LineWidth', 2, 'Color', colors(j,:));
    plot(t(1:n_nores), sigma_deg_nores, '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('$\sigma$ (deg)', 'Interpreter', 'latex');
grid on; xlim([0, t_end]);
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% --- 图7: 弹性因子 omega_2i 对比 ---
figure(7); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
hold on;
for midx = 1:M
    omega_res = squeeze(res.weights_log(1:n_res, midx, 2));
    omega_nores = squeeze(nores.weights_log(1:n_nores, midx, 2));
    plot(t(1:n_res), omega_res, '-', 'LineWidth', 1.8, 'Color', colors(midx,:));
    plot(t(1:n_nores), omega_nores, '--', 'LineWidth', 1.8, 'Color', colors(midx,:));
end
xlabel('t(s)'); ylabel('$\omega_{2,i}$', 'Interpreter', 'latex');
xlim([0, t_end]); ylim([-0.05, 1.05]); grid on;
xline(10, '--', 'T=10s', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, ...
    'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% --- 图8: 统一图例（实线=有弹性因子, 虚线=无弹性因子） ---
figure(8); clf;
set(gcf, 'Position', [200, 380, 750, 170], 'Color', 'w');
ax = axes('Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]); %#ok<LAXES>
hold on;

% 图例背景框
rectangle('Position', [0.03, 0.04, 0.94, 0.3], 'FaceColor', 'w', ...
     'LineWidth', 0.8);

n_col = M;
col_w = 0.8 / n_col;
start_x = 0.12;

% 右侧行标注
text(0.15, 0.255, 'With \eta_1:',   'FontSize', 12, 'FontName', 'Times New Roman', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
text(0.15, 0.105, 'Without \eta_1:', 'FontSize', 12, 'FontName', 'Times New Roman', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');

for midx = 1:n_col
    cx = start_x + (midx - 0.4) * col_w;   % 列中心

    % 上行：实线 = With resilience
    line([cx-0.06, cx], [0.255, 0.255], 'LineWidth', 2.5, 'Color', colors(midx,:));
    text(cx+0.02, 0.255, sprintf('Missile %d', midx), ...
        'FontSize', 12, 'FontName', 'Times New Roman', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');

    % 下行：虚线 = Without resilience
    line([cx-0.06, cx], [0.105, 0.105], 'LineWidth', 2.5, ...
        'LineStyle', '--', 'Color', colors(midx,:));
    text(cx+0.02, 0.105, sprintf('Missile %d', midx), ...
        'FontSize', 12, 'FontName', 'Times New Roman', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
end
set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% --- 图9: 累计网络断联时间 ---
figure(9); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
hold on;
plot(t(1:n_res), res.cumulative_disconnect_time_log(1:n_res), '-', 'LineWidth', 2);
plot(t(1:n_nores), nores.cumulative_disconnect_time_log(1:n_nores), '--', 'LineWidth', 2);
xlabel('Time $t$ (s)', 'Interpreter', 'latex');
ylabel('Cumulative Disconnect Time (s)', 'Interpreter', 'latex');
title('Cumulative Network Disconnection Time $\Sigma t_c$', 'Interpreter', 'latex');
xlim([0, t_end]); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% --- 图10: 拉普拉斯矩阵秩 ---
figure(10); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
hold on;
plot(t(1:n_res), res.rank_L_log(1:n_res), '-', 'LineWidth', 2);
plot(t(1:n_nores), nores.rank_L_log(1:n_nores), '--', 'LineWidth', 2);
yline(N-1, 'r--', 'LineWidth', 1.5);
xlabel('Time $t$ (s)', 'Interpreter', 'latex');
ylabel('rank(L)', 'Interpreter', 'latex');
title('Rank of Laplacian Matrix', 'Interpreter', 'latex');
xlim([0, t_end]); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% --- 图11: 累计 Control Effort 对比（J_i = Σ(Ay²+Az²)·dt） ---
figure(11); clf;
set(gcf, 'Position', [50, 50, 800, 600]);
hold on;
for j = 1:M
    % 有弹性因子
    cum_res = zeros(n_res, 1);
    for i = 1:n_res
        if ~isnan(res.Ay(i, j))
            effort = res.Ay(i, j)^2 + res.Az(i, j)^2;
            if i == 1
                cum_res(i) = effort * dt;
            else
                cum_res(i) = cum_res(i-1) + effort * dt;
            end
        else
            cum_res(i) = cum_res(i-1);
        end
    end
    plot(t(1:n_res), cum_res, '-', 'LineWidth', 2, 'Color', colors(j,:));

    % 无弹性因子
    cum_nores = zeros(n_nores, 1);
    for i = 1:n_nores
        if ~isnan(nores.Ay(i, j))
            effort = nores.Ay(i, j)^2 + nores.Az(i, j)^2;
            if i == 1
                cum_nores(i) = effort * dt;
            else
                cum_nores(i) = cum_nores(i-1) + effort * dt;
            end
        else
            cum_nores(i) = cum_nores(i-1);
        end
    end
    plot(t(1:n_nores), cum_nores, '--', 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('$\int_{0}^{t} \|A_i\|^2\, \mathrm{d}\tau$ (m$^2$/s$^3$)', 'Interpreter', 'latex');
xlim([0, t_end]); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

%% ========== 导出所有图片为高清晰度 PDF ==========
output_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~isfolder(output_dir)
    mkdir(output_dir);
end

fig_list = 1:11;
fig_names = {'resilientcompare_3D_Trajectory', 'resilientcompare_tgo_and_Range', 'resilientcompare_MultiChannel_DoS', ...
             'resilientcompare_Ay_Az', 'resilientcompare_Observer_Error', 'resilientcompare_Lead_Angle_Sigma', ...
             'resilientcompare_Resilience_Factor_Omega', 'resilientcompare_Legend', ...
             'resilientcompare_Cumulative_Disconnect_Time', 'resilientcompare_Rank_Laplacian', ...
             'resilientcompare_Control_Effort'};
fig_sizes = [800, 600; 800, 600; 800, 600; 800, 600; ...
             800, 900; 800, 600; 800, 600; 750, 170; 800, 600; 800, 600; ...
             800, 600];

for f_idx = 1:length(fig_list)
    fig_handle = figure(fig_list(f_idx));
    set(fig_handle, 'Position', [50, 50, fig_sizes(f_idx,1), fig_sizes(f_idx,2)]);
    set(fig_handle, 'PaperPositionMode', 'auto');
    pdf_path = fullfile(output_dir, [fig_names{f_idx}, '.pdf']);
    exportgraphics(fig_handle, pdf_path, 'Resolution', 600, 'ContentType', 'vector');
    fprintf('Exported: %s\n', pdf_path);
end
fprintf('All figures exported to %s/\n', output_dir);

return;

%% ========== 局部函数：单次仿真运行 ==========
function sim = run_simulation(use_resilience, tf, dt, t, Vm, N, M)
% use_resilience = true  → omega_2i = psi_i * phi_i (考虑弹性因子)
% use_resilience = false → omega_2i = 1           (不考虑弹性因子)

sigma_max = 5*pi/180;
alpha = 5;
beta = 5;
p = 0.8;
q = 1.2;
m = 1.5;
miu = 0.3;
v = 0.7;
n = 3;
m1 = 0.5;
varpi = 2; %#ok<NASGU>
lambda_info = 0.0008;
obs = obstacles(1, 1, 1);

% 通信拓扑
a_base = [1,1,0,1; 1,1,1,0; 0,1,1,1; 1,0,1,1];
a_now = [1,1,0,1; 1,1,1,0; 0,1,1,1; 1,0,1,1];

% DoS 参数
zeta_ij = 1*ones(M); %#ok<NASGU>
mu_ij = 0.52*ones(M); %#ok<NASGU>
kappa_ij = 2*ones(M); %#ok<NASGU>
nu_ij = 5*ones(M); %#ok<NASGU>
attack_prob = 0.1*ones(M); %#ok<NASGU>
dos_downtime = zeros(M); %#ok<NASGU>
dos_active = zeros(M); %#ok<NASGU>
dos_event_count = zeros(M); %#ok<NASGU>
dos_last_event_time = zeros(M); %#ok<NASGU>
t0 = 0; %#ok<NASGU>

load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');

x = [12500,-45*pi/180,45*pi/180,30*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-45*pi/180,45*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,30*pi/180,-30*pi/180];

% 初始化观测器
z_observer = zeros(M, M*5);
for i = 1:M
    for j = 1:M
        if i == j
            z_observer(i, (5*(j-1)+1):5*j) = x((5*(j-1))+1:5*j);
        else
            z_observer(i, 5*(j-1)+1) = x(5*(j-1)+1) + randi([100, 1000]);
            z_observer(i, 5*(j-1)+2) = x(5*(j-1)+2) + (randi([1, 10]) * pi/180);
            z_observer(i, 5*(j-1)+3) = x(5*(j-1)+3) + (randi([1, 10]) * pi/180);
            z_observer(i, 5*(j-1)+4) = x(5*(j-1)+4) + (randi([1, 10]) * pi/180);
            z_observer(i, 5*(j-1)+5) = x(5*(j-1)+5) + (randi([1, 10]) * pi/180);
        end
    end
end
z_observer_log = reshape(z_observer', 1, M*M*5);

T_safe = 5;
T = 10;
cumulative_disconnect_time = 0;

x_state = x;
rank_L_log = zeros(length(t), 1);
weights_log = zeros(length(t), M, 4);
observer_weights_log = zeros(length(t), M, 2);
cumulative_disconnect_time_log = zeros(length(t), 1);
last_psi_i = zeros(M, 1);

for i = 1:length(t)
    a_now = squeeze(a_log(i,:,:));
    dos_downtime = squeeze(dos_downtime_log(i,:,:)); %#ok<NASGU>
    dos_active = squeeze(dos_active_log(i,:,:)); %#ok<NASGU>
    dos_event_count = squeeze(dos_event_count_log(i,:,:)); %#ok<NASGU>

    tgo_matrix = zeros(M, M);
    sigma_matrix = zeros(M, M);

    for i_missile = 1:M
        for j_missile = 1:M
            if a_base(i_missile, j_missile) == 1 && a_now(i_missile, j_missile) == 0
                r_obs = z_observer(i_missile, 5*(j_missile-1)+1);
                theta_obs = z_observer(i_missile, 5*(j_missile-1)+4);
                psi_obs = z_observer(i_missile, 5*(j_missile-1)+5);
                sigma_matrix(i_missile, j_missile) = acos(cos(theta_obs) * cos(psi_obs));
                tgo_matrix(i_missile, j_missile) = r_obs * (1 + (sin(sigma_matrix(i_missile, j_missile))^2) / (2 * (2*N - 1))) / Vm(j_missile);
            else
                sigma_matrix(i_missile, j_missile) = acos(cos(x(5*(j_missile-1)+4)) * cos(x(5*(j_missile-1)+5)));
                tgo_matrix(i_missile, j_missile) = x(5*(j_missile-1)+1) * (1 + (sin(sigma_matrix(i_missile, j_missile))^2) / (2 * (2*N - 1))) / Vm(j_missile);
            end
        end
    end

    for j = 1:M
        sigma(i,j) = acos(cos(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)));
        tgo(i,j) = (x(5*(j-1)+1)) * (1 + ((sin(sigma(i,j))^2) / (2 * (2*N - 1)))) / Vm(j);
    end

    for j = 1:M
        epsilon(i,j) = Epsilon(tgo_matrix(j,:), a_base, j);
        has_disconnected = false;
        for k = 1:M
            if k ~= j && a_base(j, k) == 1 && a_now(j, k) == 0
                has_disconnected = true;
                break;
            end
        end
        if sigma(i,j) > 0.01
            Aybt(i,j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+5)) * Phi(sigma(i,j), sigma_max, n) * (alpha*sig(epsilon(i,j), p) + beta*sig(epsilon(i,j), q))) / ...
                (x(5*(j-1)+1) * tgo(i,j) * sin(sigma(i,j))^2);
            Azbt(i,j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)) * Phi(sigma(i,j), sigma_max, n) * ...
                (alpha*sig(epsilon(i,j), p) + beta*sig(epsilon(i,j), q))) / ...
                (x(5*(j-1)+1) * tgo(i,j) * sin(sigma(i,j))^2);
        else
            Aybt(i,j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+5)) * Phi(sigma(i,j), sigma_max, n) * ...
                (alpha*sig(epsilon(i,j), p) + beta*sig(epsilon(i,j), q))) / ...
                (x(5*(j-1)+1) * tgo(i,j));
            Azbt(i,j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)) * Phi(sigma(i,j), sigma_max, n) * ...
                (alpha*sig(epsilon(i,j), p) + beta*sig(epsilon(i,j), q))) / ...
                (x(5*(j-1)+1) * tgo(i,j));
        end
    end

    L_mat = compute_laplacian(a_now);
    rank_L = rank(L_mat);
    rank_L_log(i) = rank_L;
    if rank_L < N-1
        cumulative_disconnect_time = cumulative_disconnect_time + dt;
    end
    cumulative_disconnect_time_log(i) = cumulative_disconnect_time;
    kappa_observer = 1 / (T_safe - cumulative_disconnect_time);

    for j = 1:M
        R_ItoL = [cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), -sin(x(5*(j-1)+2));
            -sin(x(5*(j-1)+3)), cos(x(5*(j-1)+3)), 0;
            sin(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), sin(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), cos(x(5*(j-1)+2))];
        R_LtoV = [cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), cos(x(5*(j-1)+4))*sin(x(5*(j-1)+5)), -sin(x(5*(j-1)+4));
            -sin(x(5*(j-1)+5)), cos(x(5*(j-1)+5)), 0;
            sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), sin(x(5*(j-1)+4))*sin(x(5*(j-1)+5)), cos(x(5*(j-1)+4))];
        R_VtoL = R_LtoV';
        R_LtoI = R_ItoL';

        [psi_i, has_connections] = information_credibility_factor(z_observer, x, a_now, j, lambda_info, last_psi_i(j), 0.3);
        if ~has_connections
            psi_i = last_psi_i(j);
        else
            last_psi_i(j) = psi_i;
        end
        phi_i = 1;

        if use_resilience
            omega_2i = psi_i * phi_i;  % 弹性因子
        else
            omega_2i = 1;             % 无弹性因子
        end

        weights_log(i, j, :) = [0, omega_2i, phi_i, psi_i];

        Ay_png = -N*Vm(j)^2*sin(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Aybt(i,j);
        Az_png = -N*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Azbt(i,j);

        a_N = R_LtoI * R_VtoL * [0; Ay_png; Az_png];
        a_S = [0; 0; 0];
        A = a_N + a_S;

        A_V(i,j,:) = R_LtoV * R_ItoL * A;
        Ay(i,j) = A_V(i,j,2);
        Az(i,j) = A_V(i,j,3);

        x(5*(j-1)+1:5*(j-1)+5) = RK4(i, x(5*(j-1)+1:5*(j-1)+5)', Ay(i,j), Az(i,j), dt, Vm(j));
        X(i,j) = -x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*cos(x(5*(j-1)+3));
        Y(i,j) = -x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*sin(x(5*(j-1)+3));
        Z(i,j) = -x(5*(j-1)+1).*sin(x(5*(j-1)+2));

        if j == M
            x_state = [x_state; x]; %#ok<AGROW>

            mu_observer = T / (T - t(i));
            if i == 1
                [Ay_obs, Az_obs, last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, 1, 1, lambda_info, x, zeros(M, M), use_resilience);
            else
                [Ay_obs, Az_obs, last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, 1, 1, lambda_info, x, last_psi_i_obs{i-1}, use_resilience);
            end

            z_observer = observer_RK4(t(i), z_observer, a_now, kappa_observer, mu_observer, m1, ...
                Ay_obs, Az_obs, dt, Vm', T);

            for i_obs = 1:M
                z_observer(i_obs, 5*(i_obs-1)+1:5*i_obs) = x(5*(i_obs-1)+1:5*i_obs);
            end

            z_observer_log = [z_observer_log; reshape(z_observer', 1, M*M*5)]; %#ok<AGROW>

            for i_obs = 1:M
                [psi_i_obs, ~] = information_credibility_factor(z_observer, x, a_now, i_obs, lambda_info);
                if use_resilience
                    observer_weights_log(i, i_obs, :) = [0, psi_i_obs];
                else
                    observer_weights_log(i, i_obs, :) = [0, 1];
                end
            end
        end
        if x(5*(j-1)+1) <= 0
            break;
        end
    end
    if x(5*(j-1)+1) <= 0
        break;
    end
end

% 后处理：截断命中后的数据
for j = 1:M
    r_col = 5*(j-1) + 1;
    hit_idx = find(x_state(:, r_col) <= 0, 1, 'first');
    if ~isempty(hit_idx)
        x_state(hit_idx:end, r_col:r_col+4) = NaN;
        tgo(hit_idx:end, j) = NaN;
        sigma(hit_idx:end, j) = NaN;
        Ay(hit_idx:end, j) = NaN;
        Az(hit_idx:end, j) = NaN;
        X(hit_idx:end, j) = NaN;
        Y(hit_idx:end, j) = NaN;
        Z(hit_idx:end, j) = NaN;
        weights_log(hit_idx:end, j, :) = NaN;
    end
end

% 打包输出到结构体
actual_steps = size(x_state, 1) - 1;
sim.X = X;
sim.Y = Y;
sim.Z = Z;
sim.tgo = tgo;
sim.sigma = sigma;
sim.Ay = Ay;
sim.Az = Az;
sim.x_state = x_state;
sim.weights_log = weights_log;
sim.observer_weights_log = observer_weights_log;
sim.z_observer_log = z_observer_log;
sim.cumulative_disconnect_time_log = cumulative_disconnect_time_log;
sim.rank_L_log = rank_L_log;
sim.actual_steps = actual_steps;
sim.t_end = t(actual_steps);
end

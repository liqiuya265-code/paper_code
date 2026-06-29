% plot_resilience_comparison.m
% 弹性因子对比脚本：psi only vs full (psi*phi) vs none (omega=1)
% 三种模式均开启避障，同一导弹同色，不同模式不同线型，全部在一张图中
clc;clear

%% ===== 公共参数设置 =====
tf=100; dt=0.1; t=0:dt:tf;
Vm=[300,300,300,300]; N=4; M=4;
sigma_max=13*pi/180;
alpha=5; beta=5; p=0.8; q=1.2; m=1.5; miu=0.3; v=0.7; n=3; m1=0; varpi=2;
lambda_info = 0.0007;

% 通信拓扑
a_base=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
a_now=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];

% 加载DoS场景
load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');

% 初始状态
x0=[12500,-45*pi/180,45*pi/180,30*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-45*pi/180,45*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,45*pi/180,30*pi/180,-30*pi/180];

T_safe = 4.6; T = 10;

% 障碍物参数
d_safe = 30; kappa1 = 1; kappa2 = 1;
omega_env_i = 1.75*ones(1, M); n_env = 2;
use_obstacle_avoidance = true;

% 三种模式
modes = {'both','psi', 'none'};
mode_labels = {'Full ($\eta_1\eta_2$)','$\psi$ only ($\eta_1$)', 'None ($\omega=1$)'};
n_modes = length(modes);

%% ===== 运行三次仿真 =====
results = cell(n_modes, 1);
for mi = 1:n_modes
    fprintf('Running simulation: %s ...\n', modes{mi});
    rng(1);
    [X_tmp, Y_tmp, Z_tmp, tgo_tmp, sigma_tmp, Ay_tmp, Az_tmp, ...
        x_state_tmp, weights_log_tmp, z_observer_log_tmp, len_tmp] = ...
        run_single_sim(t, dt, Vm, N, M, sigma_max, alpha, beta, p, q, m, miu, v, n, ...
            a_base, a_now, a_log, dos_downtime_log, dos_active_log, dos_event_count_log, ...
            x0, T_safe, T, lambda_info, d_safe, kappa1, kappa2, omega_env_i, n_env, ...
            use_obstacle_avoidance, m1, modes{mi});
    results{mi} = struct('X', X_tmp, 'Y', Y_tmp, 'Z', Z_tmp, ...
        'tgo', tgo_tmp, 'sigma', sigma_tmp, 'Ay', Ay_tmp, 'Az', Az_tmp, ...
        'x_state', x_state_tmp, 'weights_log', weights_log_tmp, ...
        'z_observer_log', z_observer_log_tmp, 'len', len_tmp);
end
disp('All simulations complete.');

%% ===== 配色与线型 =====
% 导弹 → 颜色（同导弹同色）
colors = lines(M);                 % 4 colors, 与 plot_obstacle_comparison 一致
% 模式 → 线型（同模式同线型）
mode_styles = {'-','--', '-.'};   % psi=虚线, both=实线, none=点划线
lw = 2;

% 统一最短长度
len_all = min(cellfun(@(c) c.len, results));

% 障碍物对象（用于 3D 绘图）
obs_plot = obstacles(d_safe, kappa1, kappa2);
obs_plot.add_spherical_obstacle([-3000, -4600, 4000], 500);
obs_plot.add_cylindrical_obstacle([-4500, -1800, 0], 500, [0, 0, 1]);
obs_plot.add_spherical_obstacle([-3500, -3000, 7500], 500);
obs_plot.add_cylindrical_obstacle([-2000, -2800, 0], 500, [0, 0, 1]);

%% ===== 图1：3D轨迹（12条曲线 + 障碍物）=====
figure(1)
set(gcf, 'Position', [100, 100, 800, 600]);
hold on;
for mi = 1:n_modes
    r = results{mi};
    for j = 1:M
        plot3(r.X(1:r.len, j), r.Y(1:r.len, j), r.Z(1:r.len, j), ...
            mode_styles{mi}, 'LineWidth', lw, 'Color', colors(j,:));
    end
end
obs_plot.plot_obstacles();
plot3(0,0,0,'ko','MarkerSize',10,'LineWidth',2);
text(0,0,0,' Target','FontSize',11);
zlim([0 9000]);
set(gca, 'XDir', 'reverse', 'YDir', 'reverse');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; view(135, 20);

% ---- 四个障碍物局部放大截面（嵌入 3D 轨迹图四角）----
% 障碍物信息（与 obs_plot 一致）
obs_info = {
    struct('type','sphere',  'center',[-3000, -4600, 4000], 'R',500, 'title','Obs 1 (sphere)'), ...
    struct('type','cylinder','center',[-4500, -1800, 0],    'R',500, 'title','Obs 2 (cylinder)'), ...
    struct('type','sphere',  'center',[-3500, -3000, 7500], 'R',500, 'title','Obs 3 (sphere)'), ...
    struct('type','cylinder','center',[-2000, -2800, 0],    'R',500, 'title','Obs 4 (cylinder)')
};
inset_pos = {[0.2 0.58 0.18 0.18], [0.7 0.58 0.18 0.18], ...
             [0.2 0.25 0.18 0.18], [0.7 0.25 0.18 0.18]};
inset_axes = gobjects(1, 4);
zone_labels = gobjects(1, 4);
inset_tick_font_size = 12;
zone_label_font_size = 12;
view_range = 2000;  % 局部放大范围 ±2000m
z_view_pad = 3000;
for oi = 1:4
    p_o = obs_info{oi}.center;
    R = obs_info{oi}.R;
    is_cylinder = strcmp(obs_info{oi}.type, 'cylinder');

    % 先收集附近导弹轨迹用于确定圆柱 Z 范围
    z_all = [];
    if is_cylinder
        for mi = 1:n_modes
            r_mi = results{mi};
            for j = 1:M
                d2_xy = (r_mi.X(1:r_mi.len,j) - p_o(1)).^2 + ...
                        (r_mi.Y(1:r_mi.len,j) - p_o(2)).^2;
                in_r = d2_xy < (view_range + R)^2;
                if any(in_r)
                    z_all = [z_all; r_mi.Z(in_r,j)]; %#ok<AGROW>
                end
            end
        end
    end
    if is_cylinder && ~isempty(z_all)
        z_mid = (min(z_all) + max(z_all)) / 2;
    else
        z_mid = p_o(3);
    end

    inset_ax = axes('Position', inset_pos{oi});
    inset_axes(oi) = inset_ax;
    hold(inset_ax, 'on');

    % 导弹 3D 轨迹（局部，三模式用各自线型）
    % 俯视图用 XY 距离筛选，避免 Z 方向差异导致轨迹被误裁剪
    for mi = 1:n_modes
        r_mi = results{mi};
        for j = 1:M
            xx = r_mi.X(1:r_mi.len, j);
            yy = r_mi.Y(1:r_mi.len, j);
            zz = r_mi.Z(1:r_mi.len, j);
            d2_xy = (xx - p_o(1)).^2 + (yy - p_o(2)).^2;
            in_range = d2_xy < (view_range + R)^2;
            if any(in_range)
                plot3(inset_ax, xx(in_range), yy(in_range), zz(in_range), ...
                    mode_styles{mi}, 'LineWidth', lw, 'Color', colors(j,:));
            end
        end
    end

    % 障碍物自身（不含安全边界）
    if ~is_cylinder
        [Xs, Ys, Zs] = sphere(20);
        surf(inset_ax, Xs*R + p_o(1), Ys*R + p_o(2), Zs*R + p_o(3), ...
            'FaceColor', [0.85 0.20 0.15], 'FaceAlpha', 0.45, ...
            'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    else
        th_c = linspace(0, 2*pi, 100);
        fill3(inset_ax, p_o(1) + R*cos(th_c), p_o(2) + R*sin(th_c), ...
            z_mid * ones(1,100), [0.85 0.20 0.15], ...
            'FaceAlpha', 0.45, 'EdgeColor', [0.55 0.08 0.05], 'LineWidth', 1.2);
    end

    xlim(inset_ax, p_o(1) + [-view_range, view_range]);
    ylim(inset_ax, p_o(2) + [-view_range, view_range]);
    if is_cylinder && ~isempty(z_all)
        z_pad = z_view_pad;
        zlim(inset_ax, [min(z_all)-z_pad, max(z_all)+z_pad]);
    else
        zlim(inset_ax, p_o(3) + [-view_range-z_view_pad, view_range+z_view_pad]);
    end
    view(inset_ax, 0, 90);  % 俯视图
    set(inset_ax, 'XDir', 'reverse', 'YDir', 'reverse');
    set(inset_ax, 'FontName', 'Times New Roman', 'FontSize', inset_tick_font_size);
    box(inset_ax, 'on');

    label_pos = [inset_pos{oi}(1)+0.006, inset_pos{oi}(2)+inset_pos{oi}(4)-0.04, 0.08, 0.035];
    zone_labels(oi) = annotation(gcf, 'textbox', label_pos, ...
        'String', sprintf('Zone %d', oi), ...
        'FitBoxToText', 'on', 'LineStyle', 'none', ...
        'BackgroundColor', 'none', 'Margin', 1, ...
        'FontName', 'Times New Roman', 'FontSize', zone_label_font_size, ...
        'FontWeight', 'bold');
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);
set(inset_axes, 'FontName', 'Times New Roman', 'FontSize', inset_tick_font_size);
set(zone_labels, 'FontName', 'Times New Roman', 'FontSize', zone_label_font_size, ...
    'FontWeight', 'bold');

%% ===== 图2：tgo 和距离 r（2×1 子图，每张12条曲线）=====
figure(2)
set(gcf, 'Position', [100, 100, 800, 600]);

% --- tgo ---
subplot(2,1,1); hold on;
for mi = 1:n_modes
    r = results{mi};
    for j = 1:M
        plot(t(1:r.len), r.tgo(1:r.len, j), mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
ylabel('t_{go} (s)'); xlabel('t(s)'); grid on;

% --- 距离 r ---
subplot(2,1,2); hold on;
for mi = 1:n_modes
    r = results{mi};
    rr = r.x_state(1:r.len, 1:5:20);
    for j = 1:M
        plot(t(1:r.len), rr(:, j), mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
xlabel('t(s)'); ylabel('R (m)'); grid on;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

%% ===== 图3：加速度 Ay, Az（2×1 子图，每张12条曲线）=====
figure(3)
set(gcf, 'Position', [100, 100, 800, 600]);

subplot(2,1,1); hold on;
for mi = 1:n_modes
    r = results{mi};
    for j = 1:M
        plot(t(1:r.len), r.Ay(1:r.len, j), mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
ylabel('A_y (m/s^2)'); grid on;

subplot(2,1,2); hold on;
for mi = 1:n_modes
    r = results{mi};
    for j = 1:M
        plot(t(1:r.len), r.Az(1:r.len, j), mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
xlabel('t(s)'); ylabel('A_z (m/s^2)'); grid on;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

%% ===== 图4：前置角 sigma（12条曲线）=====
figure(4)
set(gcf, 'Position', [100, 100, 800, 600]);
hold on;
for mi = 1:n_modes
    r = results{mi};
    for j = 1:M
        plot(t(1:r.len), rad2deg(r.sigma(1:r.len, j)), mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
xlabel('t(s)'); ylabel('\sigma (deg)'); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

%% ===== 图5：权重因子对比（3×1 子图: phi, psi, omega）=====
% 各模式激活的因子: psi模式→仅psi; both模式→psi+phi; none模式→无
phi_active = [true,false, false];  % phi 仅在 both 中激活
psi_active = [ true,true,  false]; % psi 在 psi 和 both 中激活

figure(5)
set(gcf, 'Position', [100, 80, 800, 600], 'Color', 'w');

% --- 子图1：phi_i（环境安全因子, eta_2）---
subplot(3,1,1); hold on;
for mi = 1:n_modes
    r = results{mi};
    wlog = r.weights_log;
    for j = 1:M
        if phi_active(mi)
            phi_vals = squeeze(wlog(1:r.len, j, 3));
        else
            phi_vals = ones(r.len, 1);  % 未激活时恒为1
        end
        plot(t(1:r.len), phi_vals, mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
ylim([-0.05, 1.05]); ylabel('\eta_{2,i}'); xlabel('t(s)'); grid on;
xline(T, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'HandleVisibility', 'off');

% --- 子图2：psi_i（信息可信因子, eta_1）---
subplot(3,1,2); hold on;
for mi = 1:n_modes
    r = results{mi};
    wlog = r.weights_log;
    for j = 1:M
        if psi_active(mi)
            psi_vals = squeeze(wlog(1:r.len, j, 4));
        else
            psi_vals = ones(r.len, 1);  % 未激活时恒为1
        end
        plot(t(1:r.len), psi_vals, mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
ylim([-0.05, 1.05]); ylabel('\eta_{1,i}'); xlabel('t(s)'); grid on;
xline(T, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'HandleVisibility', 'off');

% --- 子图3：omega_2i = phi_i * psi_i（复合权重）---
subplot(3,1,3); hold on;
for mi = 1:n_modes
    r = results{mi};
    wlog = r.weights_log;
    for j = 1:M
        omega_vals = squeeze(wlog(1:r.len, j, 2));
        omega_vals(omega_vals <= 0) = NaN;  % 去掉零值，不显示
        plot(t(1:r.len), omega_vals, mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
ylim([-0.05, 1.05]); ylabel('\eta_{1,i} \times \eta_{2,i}'); grid on;
xlabel('t(s)');
xline(T, '--', 'T=10s', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, ...
    'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

%% ===== 图6：累积控制能量 (cumulative control effort) =====
figure(6)
set(gcf, 'Position', [100, 100, 800, 600]);
hold on;
for mi = 1:n_modes
    r = results{mi};
    cum_eff = compute_cumulative_effort(r.Ay, r.Az, r.len, M, dt);
    for j = 1:M
        plot(t(1:r.len), cum_eff(1:r.len, j), mode_styles{mi}, ...
            'LineWidth', lw, 'Color', colors(j,:));
    end
end
xlabel('t(s)'); ylabel('Cumulative control effort (m^2/s^3)'); grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

%% ===== 图7：统一图例（3行：三种模式 × 4导弹）=====
figure(7); clf;
set(gcf, 'Position', [200, 420, 800, 90], 'Color', 'w');
ax = axes('Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
hold on;

% 图例背景框
rectangle('Position', [0.03, 0.06, 0.94, 0.86], 'FaceColor', 'w', ...
     'LineWidth', 0.8);

n_col = M;
col_w = 0.75 / n_col;
start_x = 0.18;

% 三行标注（间距压缩）
row_y = [0.76, 0.48, 0.20];
row_labels = {'With $\eta_1$ and $\eta_2$:', ...
              'With $\eta_1$ only:', ...
              'Without $\eta_1$ and $\eta_2$:'};
for mi = 1:n_modes
    text(0.21, row_y(mi), row_labels{mi}, 'FontSize', 12, ...
        'FontName', 'Times New Roman', 'Interpreter', 'latex', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    for j = 1:n_col
        cx = start_x + (j - 0.4) * col_w;
        line([cx-0.06, cx], [row_y(mi), row_y(mi)], 'LineWidth', 2.5, ...
            'LineStyle', mode_styles{mi}, 'Color', colors(j,:));
        text(cx+0.02, row_y(mi), sprintf('Missile %d', j), ...
            'FontSize', 12, 'FontName', 'Times New Roman', ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
    end
end
set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

%% ===== Figure 8: Observer error, With eta_1 and eta_2 only =====
r_obs = results{1};
x_state_obs = r_obs.x_state;
z_observer_log_obs = r_obs.z_observer_log;
len_obs_err = min([r_obs.len, size(x_state_obs, 1), size(z_observer_log_obs, 1)]);
t_plot_obs = t(1:len_obs_err);

figure(8)
set(gcf, 'Position', [100, 50, 800, 600]);
roman_labels = {'i', 'ii', 'iii', 'iv'};
for i_obs = 1:M
    subplot(4, 1, i_obs)
    hold on;
    for j_target = 1:M
        if i_obs ~= j_target
            err_obs_vec = zeros(len_obs_err, 1);
            for k = 1:len_obs_err
                e_o = zeros(5, 1);
                for state_idx = 1:5
                    real_state_idx = 5*(j_target-1) + state_idx;
                    obs_idx = M*5*(i_obs-1) + 5*(j_target-1) + state_idx;
                    e_o(state_idx) = z_observer_log_obs(k, obs_idx) - x_state_obs(k, real_state_idx);
                end
                err_obs_vec(k) = norm(e_o);
            end
            plot(t_plot_obs, err_obs_vec, '-', 'LineWidth', 2, ...
                'Color', colors(j_target,:), ...
                'DisplayName', ['to Missile ', num2str(j_target)]);
        end
    end
    if i_obs == M
        xlabel('t(s)')
        xline(T, '--', 'T=10s', 'Color', [0.5 0.5 0.5], ...
            'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', ...
            'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');
    else
        xline(T, '--', 'Color', [0.5 0.5 0.5], ...
            'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    ylabel('||e_{state}||_2')
    text(0.5, 0.8, ['(', roman_labels{i_obs}, ') Missile ', ...
        num2str(i_obs), ' State Error Norm'], ...
        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
        'FontSize', 12, 'FontName', 'Times New Roman', 'Clipping', 'off');
    legend('Location', 'best')
    grid on;
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);
set(findall(gcf, 'Type', 'Legend'), 'FontSize', 12);

%% ===== 导出 PDF =====
output_dir = fileparts(mfilename('fullpath'));
if isempty(output_dir), output_dir = pwd; end

fig_names = {'trajectory', 'tgo_range', 'acceleration', 'sigma', ...
             'weights', 'control_effort', 'legend', 'observer_error'};
for fnum = 1:8
    fname = fullfile(output_dir, sprintf('resilience_%s.pdf', fig_names{fnum}));
    figure(fnum);
    exportgraphics(gcf, fname, 'Resolution', 600, 'ContentType', 'vector', 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', fname);
end
disp('All resilience comparison figures exported.');

%% ===== 辅助函数 =====
function cum_effort = compute_cumulative_effort(Ay, Az, n_steps, M, dt)
    cum_effort = zeros(n_steps, M);
    for j = 1:M
        for i = 1:n_steps
            if ~isnan(Ay(i, j))
                effort = Ay(i, j)^2 + Az(i, j)^2;
                if i == 1
                    cum_effort(i, j) = effort * dt;
                else
                    cum_effort(i, j) = cum_effort(i-1, j) + effort * dt;
                end
            elseif i > 1
                cum_effort(i, j) = cum_effort(i-1, j);
            end
        end
    end
end

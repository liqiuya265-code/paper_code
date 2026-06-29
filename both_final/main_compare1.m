clc;clear;
% ===== 对照组1：无观测器 + 有避障 =====
% 与 main.m 的区别：没有分布式状态观测器，DoS 攻击下无法补偿信息丢失
% 仅有基于连通性的简化 psi 计算和 CBF 避障

tf=100; dt=0.1; t=0:dt:tf;
Vm=[300,300,300,300]; N=4; M=4;
sigma_max=15*pi/180;
alpha=5; beta=5; p=0.8; q=1.2; m=1.5; miu=0.3; v=0.7; n=2; m1=0; varpi=2;

% 障碍物参数（与 plot_obstacle_comparison.m 保持一致）
d_safe = 100; kappa1 = 1; kappa2 = 1;
omega_env_i = 2 * ones(1, M);  % 动态 ω，首次触发避障时更新
omega_captured = false(1, M);
n_env = 2;
lambda_info = 0.001;

obs = obstacles(d_safe, kappa1, kappa2);
obs.add_spherical_obstacle([-500, -3500, 4000], 500);
obs.add_cylindrical_obstacle([-4500, -1800, 0], 500, [0, 0, 1]);
obs.add_spherical_obstacle([-2000, -500, 4500], 500);
obs.add_cylindrical_obstacle([-2000, -3000, 0], 500, [0, 0, 1]);

% 通信拓扑与 DoS 参数
a_base=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
a_now=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
zeta_ij=1*ones(M); mu_ij=0.52*ones(M);
kappa_ij=2*ones(M); nu_ij=5*ones(M);
attack_prob=0.1*ones(M);
dos_downtime=zeros(M); dos_active=zeros(M);
dos_event_count=zeros(M); dos_last_event_time=zeros(M);
t0=0; rng(1);

load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');

x=[12500,-30*pi/180,50*pi/180,45*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-30*pi/180,30*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,45*pi/180,-30*pi/180];

T_safe = 5; T = 10;
cumulative_disconnect_time = 0;

x_state = x;
rank_L_log = zeros(length(t), 1);
weights_log = zeros(length(t), M, 4);
last_psi_i = zeros(M, 1);

% 预分配
n_steps = length(t);
X = zeros(n_steps, M); Y = zeros(n_steps, M); Z = zeros(n_steps, M);
Ay = zeros(n_steps, M); Az = zeros(n_steps, M);
tgo = zeros(n_steps, M); sigma = zeros(n_steps, M);
sim_len = n_steps; break_flag = false;

for i = 1:length(t)
    a_now = squeeze(a_log(i,:,:));
    dos_downtime = squeeze(dos_downtime_log(i,:,:));
    dos_active = squeeze(dos_active_log(i,:,:));
    dos_event_count = squeeze(dos_event_count_log(i,:,:));

    % tgo_matrix: 无观测器，始终用真实状态
    tgo_matrix = zeros(M, M);
    sigma_matrix = zeros(M, M);
    for i_m = 1:M
        for j_m = 1:M
            sigma_matrix(i_m, j_m) = acos(cos(x(5*(j_m-1)+4)) * cos(x(5*(j_m-1)+5)));
            tgo_matrix(i_m, j_m) = x(5*(j_m-1)+1) * (1 + (sin(sigma_matrix(i_m, j_m))^2) / (2*(2*N-1))) / Vm(j_m);
        end
    end

    sigma_vec = zeros(1,M); tgo_vec = zeros(1,M);
    Aybt = zeros(1,M); Azbt = zeros(1,M); epsilon_vec = zeros(1,M);
    for j = 1:M
        sigma_vec(j) = acos(cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)));
        tgo_vec(j) = x(5*(j-1)+1)*(1+(sin(sigma_vec(j))^2)/(2*(2*N-1)))/Vm(j);
    end

    for j = 1:M
        epsilon_vec(j) = Epsilon(tgo_matrix(j,:), a_now, j);
        if sigma_vec(j) > 0.01
            Aybt(j) = ((2*N-1)*Vm(j)^2*sin(x(5*(j-1)+5))*Phi(sigma_vec(j),sigma_max,n)*(alpha*sig(epsilon_vec(j),p)+beta*sig(epsilon_vec(j),q)))/...
                (x(5*(j-1)+1)*tgo_vec(j)*sin(sigma_vec(j))^2);
            Azbt(j) = ((2*N-1)*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))*Phi(sigma_vec(j),sigma_max,n)*(alpha*sig(epsilon_vec(j),p)+beta*sig(epsilon_vec(j),q)))/...
                (x(5*(j-1)+1)*tgo_vec(j)*sin(sigma_vec(j))^2);
        else
            Aybt(j) = ((2*N-1)*Vm(j)^2*sin(x(5*(j-1)+5))*Phi(sigma_vec(j),sigma_max,n)*(alpha*sig(epsilon_vec(j),p)+beta*sig(epsilon_vec(j),q)))/...
                (x(5*(j-1)+1)*tgo_vec(j));
            Azbt(j) = ((2*N-1)*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))*Phi(sigma_vec(j),sigma_max,n)*(alpha*sig(epsilon_vec(j),p)+beta*sig(epsilon_vec(j),q)))/...
                (x(5*(j-1)+1)*tgo_vec(j));
        end
    end

    L_mat = compute_laplacian(a_now);
    rank_L = rank(L_mat);
    rank_L_log(i) = rank_L;
    if rank_L ~= N-1
        cumulative_disconnect_time = cumulative_disconnect_time + dt;
    end

    Ay_row = zeros(1,M); Az_row = zeros(1,M);
    X_row = zeros(1,M); Y_row = zeros(1,M); Z_row = zeros(1,M);

    for j = 1:M
        % 已命中导弹：冻结状态
        if x(5*(j-1)+1) <= 5
            X_row(j) = -x(5*(j-1)+1)*cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3));
            Y_row(j) = -x(5*(j-1)+1)*cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3));
            Z_row(j) = -x(5*(j-1)+1)*sin(x(5*(j-1)+2));
            weights_log(i,j,:) = [0, 0, 1, 1];
            if j==M, x_state=[x_state;x]; end
            continue;
        end

        p_i = [-x(5*(j-1)+1)*cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), ...
            -x(5*(j-1)+1)*cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), ...
            -x(5*(j-1)+1)*sin(x(5*(j-1)+2))];
        R_ItoL = [cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), -sin(x(5*(j-1)+2));
            -sin(x(5*(j-1)+3)), cos(x(5*(j-1)+3)), 0;
            sin(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), sin(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), cos(x(5*(j-1)+2))];
        R_LtoV = [cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), cos(x(5*(j-1)+4))*sin(x(5*(j-1)+5)), -sin(x(5*(j-1)+4));
            -sin(x(5*(j-1)+5)), cos(x(5*(j-1)+5)), 0;
            sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), sin(x(5*(j-1)+4))*sin(x(5*(j-1)+5)), cos(x(5*(j-1)+4))];
        R_VtoL = R_LtoV'; R_LtoI = R_ItoL';
        v_i = R_LtoI * R_VtoL * [Vm(j);0;0];

        % 环境安全因子（避障）
        [phi_i, r_ratio] = environmental_safety_factor(obs, p_i, omega_env_i(j), n_env);
        % 简化 psi：基于网络连通性（无观测器）
        has_connections = false;
        for k = 1:M
            if k ~= j && a_base(j,k)==1 && a_now(j,k)==1
                has_connections = true; break;
            end
        end
        if has_connections
            psi_i = 1; last_psi_i(j) = psi_i;
        else
            psi_i = last_psi_i(j);
        end
        omega_2i = psi_i * phi_i;
        Ay_png = -N*Vm(j)^2*sin(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Aybt(j);
        Az_png = -N*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Azbt(j);

        a_N = R_LtoI * R_VtoL * [0; Ay_png; Az_png];

        % CBF 避障
        [avoidance_force, obstacle_detected] = obs.compute_obstacle_avoidance(p_i', v_i, a_N);
        F_i = double(obstacle_detected);
        % 首次触发避障时捕获 ω = r_actual / R
        if F_i == 1 && ~omega_captured(j)
            omega_env_i(j) = r_ratio;
            omega_captured(j) = true;
        end
        a_S = avoidance_force;
        A_ctrl = a_N + a_S;

        weights_log(i,j,:) = [F_i, omega_2i, phi_i, psi_i];

        A_V = R_LtoV * R_ItoL * A_ctrl;
        Ay_row(j) = A_V(2); 
        
        Az_row(j) = A_V(3);

        x(5*(j-1)+1:5*(j-1)+5) = RK4(i, x(5*(j-1)+1:5*(j-1)+5)', Ay_row(j), Az_row(j), dt, Vm(j));

        X_row(j) = -x(5*(j-1)+1)*cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3));
        Y_row(j) = -x(5*(j-1)+1)*cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3));
        Z_row(j) = -x(5*(j-1)+1)*sin(x(5*(j-1)+2));

        if j == M
            x_state = [x_state; x];
        end
    end  % for j

    % 全部命中才终止
    all_hit = all(x(1:5:5*M) <= 5);
    if all_hit, sim_len = i; break_flag = true; end

    tgo(i,:) = tgo_vec; sigma(i,:) = sigma_vec;
    Ay(i,:) = Ay_row; Az(i,:) = Az_row;
    X(i,:) = X_row; Y(i,:) = Y_row; Z(i,:) = Z_row;

    if break_flag, break; end
end

%% ===== 绘图 =====
colors = lines(M);
len_plot = sim_len;
t_plot = t(1:len_plot);
fig_sz = [100, 100, 800, 600];

% 计算每个导弹的命中时刻索引（r <= 5 即视为命中）
r_all = x_state(1:len_plot, 1:5:20);
hit_idx = zeros(1, M);
for j = 1:M
    h = find(r_all(:, j) <= 5, 1, 'first');
    if isempty(h), hit_idx(j) = len_plot; else, hit_idx(j) = h; end
end

% ---- 图1：3D 轨迹 + 障碍物 ----
figure(1); set(gcf, 'Position', [50, 300, 800, 600]);
hold on;
for j = 1:M
    plot3(X(1:hit_idx(j), j), Y(1:hit_idx(j), j), Z(1:hit_idx(j), j), ...
        'LineWidth', 2, 'Color', colors(j,:), 'DisplayName', sprintf('M%d', j));
end
set(gca, 'XDir', 'reverse', 'YDir', 'reverse');
plot3(0,0,0,'ko','MarkerSize',10,'LineWidth',2); text(0,0,0,' Target','FontSize',15);
obs.plot_obstacles();
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; view(135, 20); 
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman','FontSize',18);

% ---- 图2：tgo + 距离 r（2×1）----
figure(2); set(gcf, 'Position', fig_sz);
subplot(2,1,1); hold on;
for j = 1:M
    plot(t_plot(1:hit_idx(j)), tgo(1:hit_idx(j), j), 'LineWidth', 2, 'Color', colors(j,:));
end
ylabel('t_{go} (s)'); grid on; set(gca,'FontName','Times New Roman');

% tgo 局部放大 inset: x [42,45], y [0,2]
inset_ax = axes('Position', [0.7, 0.7, 0.18, 0.18]);
hold(inset_ax, 'on');
tgo_trim = tgo(1:len_plot, :);
t_plot_col = t_plot(:);
for j = 1:M
    idx_zoom = t_plot_col >= 42 & t_plot_col <= 45 & (1:len_plot)' <= hit_idx(j);
    plot(inset_ax, t_plot_col(idx_zoom), tgo_trim(idx_zoom, j), ...
        'LineWidth', 1.5, 'Color', colors(j,:));
end
hold(inset_ax, 'off');
xlim(inset_ax, [42 45]);
ylim(inset_ax, [0 2]);
set(inset_ax, 'FontName', 'Times New Roman', 'FontSize', 8);
box(inset_ax, 'on');

subplot(2,1,2); hold on;
for j = 1:M
    plot(t_plot(1:hit_idx(j)), r_all(1:hit_idx(j), j), 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('R (m)'); grid on;
set(gca,'FontName','Times New Roman');
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman','FontSize',18);
set(inset_ax, 'FontSize', 15);

% ---- 图3：Ay + Az（2×1）----
figure(3); set(gcf, 'Position', fig_sz);
subplot(2,1,1); hold on;
for j = 1:M
    plot(t_plot(1:hit_idx(j)), Ay(1:hit_idx(j), j), 'LineWidth', 2, 'Color', colors(j,:));
end
ylabel('A_y (m/s^2)'); grid on; set(gca,'FontName','Times New Roman');
subplot(2,1,2); hold on;
for j = 1:M
    plot(t_plot(1:hit_idx(j)), Az(1:hit_idx(j), j), 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('A_z (m/s^2)'); grid on;
set(gca,'FontName','Times New Roman');
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman','FontSize',18);

% ---- 图4：前置角 sigma ----
figure(4); set(gcf, 'Position', fig_sz); hold on;
sigma_deg = rad2deg(sigma(1:len_plot, 1:4));
for j = 1:M
    plot(t_plot(1:hit_idx(j)), sigma_deg(1:hit_idx(j), j), 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel('t(s)'); ylabel('\sigma (deg)'); grid on;
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman','FontSize',18);

% ---- 图5：F_i 模式选择因子（4×1）----
figure(5); set(gcf, 'Position', [350, 50, 800, 600]);
len_wt = min(size(weights_log,1), length(t)); t_wt = t(1:len_wt);
for midx = 1:M
    subplot(M,1,midx); hold on;
    F_vals = squeeze(weights_log(1:hit_idx(midx), midx, 1));
    stairs(t_wt(1:hit_idx(midx)), F_vals, '-', 'LineWidth', 1.8, 'Color', colors(midx,:));
    active_mask = F_vals > 0;
    if any(active_mask)
        tr = diff([0; active_mask; 0]);
        si = find(tr==1); ei = find(tr==-1)-1;
        for seg = 1:length(si)
            ts = t_wt(si(seg)); if si(seg)>1, ts = t_wt(si(seg)-1); end
            te = t_wt(ei(seg)); if ei(seg)<hit_idx(midx), te = t_wt(ei(seg)+1); end
            fill([ts te te ts], [0 0 1.05 1.05], colors(midx,:), 'FaceAlpha', 0.12, 'EdgeColor', 'none');
        end
    end
    ylim([-0.1, 1.1]); yticks([0 1]); yticklabels({'Coop.', 'Avoid'});
    xlabel('t(s)'); ylabel('F_i');
    title(['Missile ', num2str(midx), ' — F_i Mode Selector']);
    grid on;
end
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman','FontSize',15);
sgtitle('F_i (No Observer)', 'FontSize', 15, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

% ---- 图6：权重因子（3×1）----
figure(6); set(gcf, 'Position', [100, 80, 800, 600]);
% φ_i
subplot(3,1,1); hold on;
for midx = 1:M
    plot(t_wt(1:hit_idx(midx)), squeeze(weights_log(1:hit_idx(midx), midx, 3)), ...
        'LineWidth', 1.8, 'Color', colors(midx,:));
end
ylim([-0.05, 1.05]); ylabel('\phi_i'); grid on;
% ψ_i
subplot(3,1,2); hold on;
for midx = 1:M
    plot(t_wt(1:hit_idx(midx)), squeeze(weights_log(1:hit_idx(midx), midx, 4)), ...
        'LineWidth', 1.8, 'Color', colors(midx,:));
end
ylim([-0.05, 1.05]); ylabel('\psi_i'); grid on;
% ω_{2i}
subplot(3,1,3); hold on;
for midx = 1:M
    plot(t_wt(1:hit_idx(midx)), squeeze(weights_log(1:hit_idx(midx), midx, 2)), ...
        'LineWidth', 1.8, 'Color', colors(midx,:));
end
ylim([-0.05, 1.05]); ylabel('\omega_{2i}'); grid on;
xlabel('t (s)');
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman','FontSize',15);
sgtitle('Weights (No Observer)', 'FontSize', 15, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

% ---- 图7：DoS 攻击时间线 ----
link_list_all = [];
for r = 1:M
    for c = 1:M
        if a_base(r,c)==1 && r~=c, link_list_all = [link_list_all; r, c]; end %#ok<AGROW>
    end
end
num_links_all = size(link_list_all,1);
link_labels_all = cell(num_links_all,1);
for k = 1:num_links_all
    link_labels_all{k} = sprintf('(%d,%d)', link_list_all(k,1), link_list_all(k,2));
end
len_a = min(length(a_log), length(t));
attack_matrix = zeros(len_a, num_links_all);
for k = 1:num_links_all
    r = link_list_all(k,1); c = link_list_all(k,2);
    link_stat = squeeze(a_log(1:len_a, r, c));
    attack_matrix(:,k) = (link_stat == 0);
end

figure(7); set(gcf, 'Position', [100, 100, 750, 500]);
gap_val = 0.02; margin_bottom = 0.08; margin_top = 0.06;
avail_h = 1 - margin_bottom - margin_top - (num_links_all-1)*gap_val;
row_h = avail_h / num_links_all;
for k = 1:num_links_all
    y_bottom = margin_bottom + (num_links_all-k)*(row_h + gap_val);
    subplot('Position', [0.12, y_bottom, 0.85, row_h]); hold on;
    stairs(t(1:len_a), attack_matrix(:,k), '-', 'Color', [0.15 0.25 0.55], 'LineWidth', 1.2);
    area(t(1:len_a), attack_matrix(:,k), 'FaceColor', [0.55 0.65 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.55);
    stairs(t(1:len_a), attack_matrix(:,k), '-', 'Color', [0.10 0.20 0.50], 'LineWidth', 1.2);
    ylim([-0.15, 1.15]); yticks([0 1]); yticklabels({'Safe', 'DoS'});
    xlim([0, t(len_a)]);
    ylabel(link_labels_all{k}, 'FontSize', 15, 'FontWeight', 'bold', 'FontName', 'Times New Roman', ...
        'Rotation', 0, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    if k < num_links_all, set(gca, 'XTickLabel', []); end
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 15, 'LineWidth', 0.5);
    grid on;
end
xlabel('t (s)', 'FontSize', 15, 'FontName', 'Times New Roman');
sgtitle('Multi-channel Asynchronous DoS (No Observer)', 'FontSize', 15, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman','FontSize',15);

%% 导出 PDF
pdf_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~exist(pdf_dir, 'dir'), mkdir(pdf_dir); end
fig_names = {'trajectory', 'tgo_range', 'acceleration', 'sigma'};
for fnum = 1:4
    fname = fullfile(pdf_dir, sprintf('compare1_fig%d_%s.pdf', fnum, fig_names{fnum}));
    figure(fnum);
    exportgraphics(gcf, fname, 'ContentType', 'vector', 'BackgroundColor', 'white');
    fprintf('Saved: %s\n', fname);
end
disp('main_compare1: 4 figures exported to IEEE Fig directory.');

clc;clear;
tf=100;
dt=0.1;
t=0:dt:tf;
Vm=[300,300,300,300];
N=4;
M=4;
sigma_max=5*pi/180;
alpha=5;
beta=5;
p=0.8;
q=1.2;
m=1.5;
miu=0.3;
v=0.7;
n=3;
m1=0.5;
varpi=2;
% 无弹性因子场景：观测器保留，但不引入信息可信因子 psi 的权重调制（omega_2i=1）
lambda_info = 0.0008;
obs = obstacles(1, 1, 1);

% 通信拓扑（未受攻击前）
a_base=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
a_now=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
% DoS 参数
zeta_ij=1*ones(M);
mu_ij=0.52*ones(M);
kappa_ij=2*ones(M);
nu_ij=5*ones(M);
attack_prob=0.1*ones(M);
dos_downtime=zeros(M);
dos_active=zeros(M);
dos_event_count=zeros(M);
dos_last_event_time=zeros(M);
t0=0;
rng(1);
load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');
x=[12500,-45*pi/180,45*pi/180,30*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-45*pi/180,45*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,30*pi/180,-30*pi/180];

% 初始化分布式全局状态观测器
z_observer = zeros(M, M*5);
initial_bias_r = 500;
initial_bias_angle = 5*pi/180;

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

% kappa(t) 函数
T_safe = 5;
kappa_1 = 1;

% mu(t) 函数
T = 10;

% 初始化累计网络断联时间
cumulative_disconnect_time = 0;

x_state=x;
rank_L_log = zeros(length(t), 1);

% 初始化权重日志记录
weights_log = zeros(length(t), M, 4);  % [F_i, omega_2i, phi_i, psi_i]
observer_weights_log = zeros(length(t), M, 2);  % [F_i_obs, omega_2i_obs]
cumulative_disconnect_time_log = zeros(length(t), 1);

% 初始化最近的 psi_i 值记录
last_psi_i = zeros(M, 1);

for i=1:length(t)
    a_now = squeeze(a_log(i,:,:));
    dos_downtime = squeeze(dos_downtime_log(i,:,:));
    dos_active = squeeze(dos_active_log(i,:,:));
    dos_event_count = squeeze(dos_event_count_log(i,:,:));

    tgo_matrix = zeros(M, M);
    sigma_matrix = zeros(M, M);

    for i_missile = 1:M
        for j_missile = 1:M
            if  a_base(i_missile, j_missile) == 1 && a_now(i_missile, j_missile) == 0
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

    for j=1:M
        sigma(i,j)=acos(cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)));
        tgo(i,j)=(x(5*(j-1)+1))*(1+((sin(sigma(i,j))^2)/(2*(2*N-1))))/Vm(j);
    end


    for j=1:M
        epsilon(i,j)=Epsilon(tgo_matrix(j,:),a_base,j);
        has_disconnected = false;
        for k = 1:M
            if k ~= j && a_base(j, k) == 1 && a_now(j, k) == 0
                has_disconnected = true;
                break;
            end
        end
        if sigma(i,j) > 0.01
            Aybt(i,j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+5)) * Phi(sigma(i,j), sigma_max, n )  * (alpha*sig(epsilon(i,j), p) + beta*sig(epsilon(i,j), q))) / ...
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
    % 计算拉普拉斯矩阵并判断图的连通性
    L = compute_laplacian(a_now);
    rank_L = rank(L);
    rank_L_log(i) = rank_L;
    if rank_L < N-1
        cumulative_disconnect_time = cumulative_disconnect_time + dt;
    end
    cumulative_disconnect_time_log(i) = cumulative_disconnect_time;
    kappa_observer =1/(T_safe-cumulative_disconnect_time);
    for j=1:M
        R_ItoL = [cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)),   cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)),   -sin(x(5*(j-1)+2));
            -sin(x(5*(j-1)+3)),               cos(x(5*(j-1)+3)),                0;
            sin(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), sin(x(5*(j-1)+2))*sin(x(5*(j-1)+3)),   cos(x(5*(j-1)+2))];
        R_LtoV = [cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)),   cos(x(5*(j-1)+4))*sin(x(5*(j-1)+5)),   -sin(x(5*(j-1)+4));
            -sin(x(5*(j-1)+5)),               cos(x(5*(j-1)+5)),                0;
            sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), sin(x(5*(j-1)+4))*sin(x(5*(j-1)+5)),   cos(x(5*(j-1)+4))];
        R_VtoL = R_LtoV';
        R_LtoI = R_ItoL';
        % 计算信息可信因子 ψ_i（仅用于日志记录，不用于控制调制）
        [psi_i, has_connections] = information_credibility_factor(z_observer, x, a_now, j, lambda_info, last_psi_i(j), 0.3);
        if ~has_connections
            psi_i = last_psi_i(j);
        else
            last_psi_i(j) = psi_i;
        end
        phi_i = 1;
        omega_2i = 1;  % 无弹性因子：偏置项以满强度施加
        % 记录权重日志
        weights_log(i, j, :) = [0, omega_2i, phi_i, psi_i];
        % 基础PNG加速度（名义控制，无弹性因子调制）
        Ay_png = -N*Vm(j)^2*sin(x(5*(j-1)+5))/x(5*(j-1)+1)-omega_2i*Aybt(i,j);
        Az_png = -N*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))/x(5*(j-1)+1)-omega_2i*Azbt(i,j);

        % 名义控制输入矢量 a_N
        a_N = R_LtoI * R_VtoL * [0; Ay_png; Az_png];
        a_S=[0;0;0];
        A = a_N+a_S;

        % 合并PNG和障碍物避免加速度
        A_V(i,j,:)=R_LtoV*R_ItoL*A;
        Ay(i,j)=A_V(i,j,2);
        Az(i,j)=A_V(i,j,3);

        x(5*(j-1)+1:5*(j-1)+5)=RK4(i,x(5*(j-1)+1:5*(j-1)+5)',Ay(i,j),Az(i,j),dt,Vm(j));
        X(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*cos(x(5*(j-1)+3));
        Y(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*sin(x(5*(j-1)+3));
        Z(i,j)=-x(5*(j-1)+1).*sin(x(5*(j-1)+2));
        if j==M
            x_state=[x_state;x];

            mu_observer=T/(T-t(i));
            if i==1

                [Ay_obs, Az_obs,last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, 1, 1, lambda_info, x,zeros(M, M), false);
            else
                [Ay_obs, Az_obs,last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, 1, 1, lambda_info, x,last_psi_i_obs{i-1}, false);
            end

            z_observer = observer_RK4(t(i), z_observer, a_now, kappa_observer, mu_observer, m1, ...
                Ay_obs, Az_obs, dt, Vm',T);

            % 确保每个导弹对自己的观测值始终等于真实状态值
            for i_obs = 1:M
                z_observer(i_obs, 5*(i_obs-1)+1:5*i_obs) = x(5*(i_obs-1)+1:5*i_obs);
            end

            z_observer_log = [z_observer_log; reshape(z_observer', 1, M*M*5)]; %#ok<AGROW>

            % 记录观测器权重日志（无弹性因子，omega_2i_obs=1）
            for i_obs = 1:M
                [psi_i_obs, ~] = information_credibility_factor(z_observer, x, a_now, i_obs, lambda_info);
                observer_weights_log(i, i_obs, :) = [0, 1];  % omega_2i_obs=1 (no resilience)
            end

        end
        if x(5*(j-1)+1)<=0
            break;
        end
    end
    if x(5*(j-1)+1)<=0
        break;
    end
end
%%
% 后处理：每个导弹到达目标后截断其数据
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

% ===== 保存控制量数据用于对比 =====
save('control_effort_no_res.mat', 'Ay', 'Az', 't');

figure(11)
plot3(X(:,1:4),Y(:,1:4),Z(:,1:4),'LineWidth',2,'LineStyle','-');
set(gca, 'XDir', 'reverse');
set(gca, 'YDir', 'reverse');
hold on;
plot3(0,0,0,'Marker','o','LineWidth',2)
text(0,0,0, 'Target')
xlabel("X(m)")
ylabel("Y(m)")
zlabel("Z(m)")
grid on;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% 统一时间长度
actual_steps = size(x_state, 1) - 1;
t_end = t(actual_steps);
len_tgo = actual_steps;
t_plot_tgo = t(1:len_tgo);
len_state = size(x_state, 1);
t_plot_state = t(1:len_state);
len_acc = actual_steps;
t_plot_acc = t(1:len_acc);

% 图12：tgo 与 R 合并图
figure(12)
subplot(2,1,1)
plot(t_plot_tgo, tgo(1:len_tgo,1:4), 'LineWidth', 2, 'LineStyle', '-');
ylabel("t_{go}(s)")
grid on;
xlim([0, t_end]);
set(gca, 'XTickLabel', []);

subplot(2,1,2)
plot(t_plot_state, x_state(:,1:5:20), 'LineWidth', 2, 'LineStyle', '-');
xlabel("t(s)")
ylabel("R(m)")
grid on;
xlim([0, t_end]);

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% 图14：上图 Ay，下图 Az
figure(14)
subplot(2,1,1)
plot(t_plot_acc, Ay(1:len_acc,1:4), 'LineWidth', 2, 'LineStyle', '-');
xlabel("t(s)")
ylabel("Ay(m/s^2)")
grid on;
xlim([0, t_end]);

subplot(2,1,2)
plot(t_plot_acc, Az(1:len_acc,1:4), 'LineWidth', 2, 'LineStyle', '-');
xlabel("t(s)")
ylabel("Az(m/s^2)")
grid on;
xlim([0, t_end]);

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% 图15：观测器状态收敛图
len_obs = min(size(x_state, 1), size(z_observer_log, 1));
t_plot_obs = t(1:len_obs);
roman_labels = {'i', 'ii', 'iii', 'iv'};
figure(15)
for i_obs = 1:M
    subplot(4,1,i_obs)
    hold on;
    for j_target = 1:M
        if i_obs ~= j_target
            error_norm = zeros(len_obs, 1);
            for k = 1:len_obs
                e_state = zeros(5, 1);
                for state_idx = 1:5
                    real_state_idx = 5*(j_target-1) + state_idx;
                    obs_idx = 20*(i_obs-1) + 5*(j_target-1) + state_idx;
                    e_state(state_idx) = z_observer_log(k, obs_idx) - x_state(k, real_state_idx);
                end
                error_norm(k) = norm(e_state);
            end
            plot(t_plot_obs, error_norm, 'LineWidth', 2, ...
                'DisplayName', ['to Missile ', num2str(j_target)]);
        end
    end
    ylabel("||e_{state}||_2")
    text(0.5, 0.8, ['(', roman_labels{i_obs}, ') Missile ', num2str(i_obs), ' State Error Norm'], ...
        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
        'FontSize', 12, 'FontName', 'Times New Roman', 'Clipping', 'off');
    if i_obs == M
        xlabel("t(s)")
    end
    legend('Location', 'best')
    grid on;
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);
set(findall(gcf, 'Type', 'Legend'), 'FontSize', 12);

% 图16：前置角变化图
len_sigma = actual_steps;
t_plot_sigma = t(1:len_sigma);
figure(16)
hold on;
sigma_deg = rad2deg(sigma(1:len_sigma,1:4));
plot(t_plot_sigma, sigma_deg, 'LineWidth', 2);
xlabel('t(s)', 'FontSize', 11, 'FontName', 'Times New Roman');
ylabel('$\sigma$ (deg)', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
grid on;
xlim([0, t_end]);
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% 图17：信息可信因子 psi 变化图
len_wt = actual_steps;
t_plot_wt = t(1:len_wt);
figure(17)
hold on;
for midx = 1:M
    psi_vals = squeeze(weights_log(1:len_wt, midx, 4));
    plot(t_plot_wt, psi_vals, 'LineWidth', 1.8, ...
        'DisplayName', ['Missile ', num2str(midx)]);
end
xlabel('t(s)', 'FontSize', 11, 'FontName', 'Times New Roman');
ylabel('$\eta_{1,i}$', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
xlim([0, t_end]);
ylim([-0.05, 1.05]);
grid on;
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% 图13：多通道异步DoS攻击示意图
link_list_all = [1,2; 2,3; 3,4; 4,1];
num_links_all = size(link_list_all, 1);
link_labels_all = cell(num_links_all, 1);
for k = 1:num_links_all
    link_labels_all{k} = sprintf('(%d,%d)', link_list_all(k,1), link_list_all(k,2));
end

len_a = min(length(a_log), length(t));
attack_matrix = zeros(len_a, num_links_all);
for k = 1:num_links_all
    r = link_list_all(k, 1);
    c = link_list_all(k, 2);
    link_stat = squeeze(a_log(1:len_a, r, c));
    attack_matrix(:, k) = (link_stat == 0);
end
t_plot_mc = t(1:len_a);

figure(13)
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
    yticks([0, 1]);
    yticklabels({'Safe', 'DoS'});
    xlim([0, t_plot_mc(end)]);
    ylabel(link_labels_all{k}, 'FontSize', 12, 'FontName', 'Times New Roman', ...
        'Rotation', 0, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    if k < num_links_all
        set(gca, 'XTickLabel', []);
    end
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 0.5);
    grid on;
    hold off;
end
xlabel('t(s)', 'FontSize', 12, 'FontName', 'Times New Roman');

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% 图18：图17 Trust Factor 统一图例
figure(18)
set(gcf, 'Position', [200, 420, 680, 130], 'Color', 'w');
ax = axes('Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
hold on;

rectangle('Position', [0.03, 0.04, 0.92, 0.2], 'FaceColor', 'w', ...
    'LineWidth', 0.8);

n_leg = M;
colors_leg = lines(M);
col_w = 0.88 / n_leg;
start_x = 0.03;

for midx = 1:n_leg
    cx = start_x + (midx - 0.5) * col_w;
    line([cx-0.06, cx], [0.15, 0.15], 'LineWidth', 2.5, 'Color', colors_leg(midx,:));
    text(cx+0.02, 0.15, sprintf('Missile %d', midx), ...
        'FontSize', 12, 'FontName', 'Times New Roman', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图19：累计网络断联时间
figure(19)
plot(t(1:actual_steps), cumulative_disconnect_time_log(1:actual_steps), 'b-', 'LineWidth', 2);
xlabel('Time $t$ (s)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
ylabel('Cumulative Disconnect Time (s)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('Cumulative Network Disconnection Time $\Sigma t_c$ (No Resilience)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
xlim([0, t_end]);
grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图20：拉普拉斯矩阵秩
figure(20)
hold on;
plot(t(1:actual_steps), rank_L_log(1:actual_steps), 'b-', 'LineWidth', 2);
yline(N-1, 'r--', 'LineWidth', 1.5, 'DisplayName', ['Threshold (N-1)=', num2str(N-1)]);
xlabel('Time $t$ (s)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
ylabel('rank(L)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('Rank of Laplacian Matrix (No Resilience)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
legend('rank(L)', ['N-1=', num2str(N-1)], 'Location', 'best');
xlim([0, t_end]);
grid on;
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% ===== 导出所有图片为高清晰度 PDF =====
output_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~isfolder(output_dir)
    mkdir(output_dir);
end

fig_list = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20];
fig_names = {'3D_Trajectory_no_res', 'tgo_and_Range_no_res', 'MultiChannel_DoS_no_res', 'Ay_Az_no_res', ...
             'Observer_Error_no_res', 'Lead_Angle_Sigma_no_res', 'Trust_Factor_Psi_no_res', ...
             'Trust_Factor_Psi_Legend_no_res', 'Cumulative_Disconnect_Time_no_res', 'Rank_Laplacian_no_res'};
fig_sizes = [800, 600; 800, 600; 800, 600; 800, 600; ...
             800, 600; 800, 600; 800, 600; 680, 130; 800, 600; 800, 600];

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

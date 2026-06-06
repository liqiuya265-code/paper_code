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
m1=0;
varpi=2;
% 纯DoS场景：不考虑障碍物，仅保留信息可信因子 psi
lambda_info = 0.001; % 信息可信因子衰减参数 λ
obs = obstacles(1, 1, 1); % 保留对象接口，纯DoS下不添加任何障碍物

% 通信拓扑（未受攻击前）
a_base=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
a_now=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
% DoS 参数：|D_{ij}(t0,t)| <= zeta_ij + mu_ij*(t-t0)
% 其中 mu_ij 为占空比上限（每秒允许的最大中断占比），需在 [0,1] 之间
zeta_ij=1*ones(M);      % 基础脉冲上界（秒）
mu_ij=0.52*ones(M);       % 占空比上限（秒/秒），示例设置为 50%
% DoS 频率约束：|F_{ij}(t0,t)| <= kappa_ij + (t-t0)/nu_ij
% nu_ij > 1 表示平均停留时间（连续DoS事件之间的最小时间间隔）
% kappa_ij > 0 表示干扰边界（DoS事件的初始数量）
kappa_ij=2*ones(M);       % 干扰边界，示例设置为 2
nu_ij=5*ones(M);         % 平均停留时间（秒），示例设置为 5秒，需满足 nu_ij > 1
attack_prob=0.1*ones(M); % 每个时间步的攻击触发概率（有余量才可触发）
dos_downtime=zeros(M);     % 累计中断时长
dos_active=zeros(M);       % 当前攻击状态标记
dos_event_count=zeros(M);  % DoS事件发生次数
dos_last_event_time=zeros(M); % 上次DoS事件发生的时间
t0=0;                      % 攻击起始计时
rng(1);                    % 固定随机种子便于复现实验
load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');
x=[12500,-45*pi/180,45*pi/180,30*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-45*pi/180,45*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,30*pi/180,-30*pi/180];

% 初始化分布式全局状态观测器
% 观测状态 z_observer: (M x M*5) = (4 x 20) 矩阵
% 每行 z_observer(i, :) 表示第 i 个导弹对所有导弹（包括自己）的全局观测值
% z_observer(i, 5*(j-1)+1:5*j) 表示第 i 个导弹对第 j 个导弹的观测
z_observer = zeros(M, M*5);
% 设置初始偏差（可以调整偏差大小）
initial_bias_r = 500;  % 距离偏差 (m)
initial_bias_angle = 5*pi/180;  % 角度偏差 (rad)

for i = 1:M  % 第 i 个导弹
    for j = 1:M  % 对第 j 个导弹的观测
        if i == j
            % 导弹对自己的观测值等于真实状态值
            z_observer(i, (5*(j-1)+1):5*j) = x((5*(j-1))+1:5*j);
        else
            % 对其他导弹的观测值有初始偏差
            z_observer(i, 5*(j-1)+1) = x(5*(j-1)+1) + randi([100, 1000]);  % r: 距离
            z_observer(i, 5*(j-1)+2) = x(5*(j-1)+2) + (randi([1, 10]) * pi/180);  % theta_L: 俯仰角
            z_observer(i, 5*(j-1)+3) = x(5*(j-1)+3) + (randi([1, 10]) * pi/180);  % psi_L: 偏航角
            z_observer(i, 5*(j-1)+4) = x(5*(j-1)+4) + (randi([1, 10]) * pi/180);  % theta: 俯仰角速度相关
            z_observer(i, 5*(j-1)+5) = x(5*(j-1)+5) + (randi([1, 10]) * pi/180);  % psi: 偏航角速度相关
        end
    end
end
z_observer_log = reshape(z_observer', 1, M*M*5);  % 保存观测器状态历史（行向量格式）

% kappa(t) 函数：kappa(t) = kappa_1 / [T_safe - Σt_c(t0,t)]^m
% 其中 Σt_c 是累计网络断联时间（rank_L ~= N-1 的累计时间）
T_safe = 1.5;  % 安全时间参数
kappa_1 = 1;  % kappa 的系数，可根据需要调整

% mu(t) 函数：mu(t; t0, T) = T / (T + t0 - t)
T = 10;  % 时间参数 T（秒），可根据需要调整

% 初始化累计网络断联时间
cumulative_disconnect_time = 0;  % 累计网络断联时间 Σt_c

x_state=x;
% DoS攻击记录从 dos_scenario.mat 加载，无需重新初始化
rank_L_log = zeros(length(t), 1);  % 记录拉普拉斯矩阵的秩

% 初始化权重日志记录
weights_log = zeros(length(t), M, 4);  % [F_i, omega_2i, phi_i, psi_i]
observer_weights_log = zeros(length(t), M, 2);  % [F_i_obs, omega_2i_obs]

% 初始化最近的 psi_i 值记录（用于无连接时的回退）
last_psi_i = zeros(M, 1);  % 初始化为0.5

for i=1:length(t)
    % 从预生成的DoS场景加载当前步的通信拓扑
    a_now = squeeze(a_log(i,:,:));
    % 记录DoS攻击状态（从预加载数据读取）
    dos_downtime = squeeze(dos_downtime_log(i,:,:));
    dos_active = squeeze(dos_active_log(i,:,:));
    dos_event_count = squeeze(dos_event_count_log(i,:,:));

    tgo_matrix = zeros(M, M);  % tgo_matrix(i, j) 表示导弹 i 计算导弹 j 的 tgo
    sigma_matrix = zeros(M, M);  % sigma_matrix(i, j) 表示导弹 i 计算导弹 j 的 sigma

    for i_missile = 1:M  % 第 i_missile 个导弹
        for j_missile = 1:M  % 对第 j_missile 个导弹的 tgo
            if  a_base(i_missile, j_missile) == 1 && a_now(i_missile, j_missile) == 0
                % t > T 且链路断开：使用 i_missile 对 j_missile 的观测状态计算 tgo
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
        % 计算偏置项
        % 检查是否有与 j 断开的链路，决定使用真实状态还是观测状态
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
    % 累计网络断联时间（当 rank_L ~= N-1 时，图不连通）
    if rank_L ~= N-1
        cumulative_disconnect_time = cumulative_disconnect_time + dt;  % 累计断联时间
    end
    kappa_observer =1;
    for j=1:M
        R_ItoL = [cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)),   cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)),   -sin(x(5*(j-1)+2));
            -sin(x(5*(j-1)+3)),               cos(x(5*(j-1)+3)),                0;
            sin(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), sin(x(5*(j-1)+2))*sin(x(5*(j-1)+3)),   cos(x(5*(j-1)+2))];
        R_LtoV = [cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)),   cos(x(5*(j-1)+4))*sin(x(5*(j-1)+5)),   -sin(x(5*(j-1)+4));
            -sin(x(5*(j-1)+5)),               cos(x(5*(j-1)+5)),                0;
            sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), sin(x(5*(j-1)+4))*sin(x(5*(j-1)+5)),   cos(x(5*(j-1)+4))];
        R_VtoL = R_LtoV';
        R_LtoI = R_ItoL';
        % 计算信息可信因子 ψ_i
        [psi_i, has_connections] = information_credibility_factor(z_observer, x, a_now, j, lambda_info);
        if ~has_connections
            psi_i = last_psi_i(j);  % 使用最近有连接时刻的 psi_i
        else
            last_psi_i(j) = psi_i;  % 更新最近的 psi_i
        end
        omega_2i=psi_i;
        % 记录权重日志: F_i=0 (纯DoS无避障), phi_i=1 (无障碍物)
        weights_log(i, j, :) = [0, omega_2i, 1, psi_i];
        % 基础PNG加速度（名义控制）
        Ay_png = -N*Vm(j)^2*sin(x(5*(j-1)+5))/x(5*(j-1)+1)-omega_2i*Aybt(i,j);
        Az_png = -N*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))/x(5*(j-1)+1)-omega_2i*Azbt(i,j);

        % 名义控制输入矢量 a_N
        a_N = R_LtoI * R_VtoL * [0; Ay_png; Az_png];
        % 纯DoS场景下不考虑障碍物控制项
        a_S=[0;0;0];
        A = a_N+a_S;

        % 合并PNG和障碍物避免加速度
        A_V(i,j,:)=R_LtoV*R_ItoL*A;
        Ay(i,j)=A_V(i,j,2);
        Az(i,j)=A_V(i,j,3);
        % if abs(Ay(i,j))>1000
        %     Ay(i,j)=sign(Ay(i,j))*1000;
        % end
        % if abs(Az(i,j))>1000
        %     Az(i,j)=sign(Az(i,j))*1000;
        % end
        x(5*(j-1)+1:5*(j-1)+5)=RK4(i,x(5*(j-1)+1:5*(j-1)+5)',Ay(i,j),Az(i,j),dt,Vm(j));
        X(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*cos(x(5*(j-1)+3));
        Y(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*sin(x(5*(j-1)+3));
        Z(i,j)=-x(5*(j-1)+1).*sin(x(5*(j-1)+2));
        if j==M
            x_state=[x_state;x];

            mu_observer=T/(T-t(i));
            if i==1

                [Ay_obs, Az_obs,last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, 1, 1, lambda_info, x,zeros(M, M));
            else
                [Ay_obs, Az_obs,last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, 1, 1, lambda_info, x,last_psi_i_obs{i-1});
            end

            z_observer = observer_RK4(t(i), z_observer, a_now, kappa_observer, mu_observer, m1, ...
                Ay_obs, Az_obs, dt, Vm',T);

            % 确保每个导弹对自己的观测值始终等于真实状态值
            for i_obs = 1:M
                z_observer(i_obs, 5*(i_obs-1)+1:5*i_obs) = x(5*(i_obs-1)+1:5*i_obs);
            end

            z_observer_log = [z_observer_log; reshape(z_observer', 1, M*M*5)]; %#ok<AGROW> 保存观测器状态

            % 记录观测器权重日志
            for i_obs = 1:M
                [psi_i_obs, ~] = information_credibility_factor(z_observer, x, a_now, i_obs, lambda_info);
                observer_weights_log(i, i_obs, :) = [0, psi_i_obs];  % F_i_obs=0, omega_2i_obs=psi_i_obs
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
figure(1)
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
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 统一时间长度，避免提前命中导致绘图维度不一致
len_tgo = size(tgo, 1);
t_plot_tgo = t(1:len_tgo);
len_state = size(x_state, 1);
t_plot_state = t(1:len_state);
len_acc = size(Ay, 1);
t_plot_acc = t(1:len_acc);

% 图2：tgo 与 R 合并图（2x1 排列）
figure(2)
subplot(2,1,1)
plot(t_plot_tgo, tgo(:,1:4), 'LineWidth', 2, 'LineStyle', '-');
ylabel("t_{go}(s)")
grid on;
set(gca, 'XTickLabel', []);

subplot(2,1,2)
plot(t_plot_state, x_state(:,1:5:20), 'LineWidth', 2, 'LineStyle', '-');
xlabel("t(s)")
ylabel("R(m)")
grid on;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图4：上图 Ay，下图 Az
figure(4)
subplot(2,1,1)
plot(t_plot_acc, Ay(:,1:4), 'LineWidth', 2, 'LineStyle', '-');
xlabel("t(s)")
ylabel("Ay(m/s^2)")
grid on;

subplot(2,1,2)
plot(t_plot_acc, Az(:,1:4), 'LineWidth', 2, 'LineStyle', '-');
xlabel("t(s)")
ylabel("Az(m/s^2)")
grid on;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图5：观测器状态收敛图（1x4，每个子图对应一个观测导弹i，绘制5维状态误差范数）
len_obs = min(size(x_state, 1), size(z_observer_log, 1));
t_plot_obs = t(1:len_obs);
figure(5)
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
            plot(t_plot_obs, error_norm, 'LineWidth', 1.5, ...
                'DisplayName', ['to Missile ', num2str(j_target)]);
        end
    end
    xlabel("t(s)")
    ylabel("||e_{state}||_2")
    title(['Missile ', num2str(i_obs), ' State Error Norm'])
    legend('Location', 'best')
    grid on;
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);
% 图5 legend 单独设为 12pt
set(findall(gcf, 'Type', 'Legend'), 'FontSize', 12);

% 图6：前置角（heading error）变化图
len_sigma = size(sigma, 1);
t_plot_sigma = t(1:len_sigma);
figure(6)
hold on;
sigma_deg = rad2deg(sigma(:,1:4));
plot(t_plot_sigma, sigma_deg, 'LineWidth', 1.8);
xlabel('t(s)', 'FontSize', 11, 'FontName', 'Times New Roman');
ylabel('$\sigma$ (deg)', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
grid on;
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图7：信息可信因子 psi 变化图
len_wt = min(size(weights_log, 1), length(t));
t_plot_wt = t(1:len_wt);
figure(7)
hold on;
for midx = 1:M
    psi_vals = squeeze(weights_log(1:len_wt, midx, 4));
    plot(t_plot_wt, psi_vals, 'LineWidth', 1.8, ...
        'DisplayName', ['Missile ', num2str(midx)]);
end
xlabel('t(s)', 'FontSize', 11, 'FontName', 'Times New Roman');
ylabel('Trust Factor $\psi_i$', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
xlim([0, 45]);
ylim([-0.05, 1.05]);
grid on;
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% ===== 多通道异步DoS攻击示意图 =====
% 统计所有存在的通信链路（排除自环）
% 仅展示 4 条代表性链路
link_list_all = [1,2; 2,3; 3,4; 4,1];
num_links_all = size(link_list_all, 1);
link_labels_all = cell(num_links_all, 1);
for k = 1:num_links_all
    link_labels_all{k} = sprintf('(%d,%d)', link_list_all(k,1), link_list_all(k,2));
end

% 提取攻击状态矩阵: 行=时间步, 列=链路
len_a = min(length(a_log), length(t));
attack_matrix = zeros(len_a, num_links_all);
for k = 1:num_links_all
    r = link_list_all(k, 1);
    c = link_list_all(k, 2);
    link_stat = squeeze(a_log(1:len_a, r, c));
    attack_matrix(:, k) = (link_stat == 0);  % 1=被攻击(DoS), 0=正常(Safe)
end
t_plot_mc = t(1:len_a);

figure(3)
set(gcf, 'Position', [100, 100, 750, 500]);

% 每个链路一个子图，纵向堆叠，共享x轴，紧凑排列
gap_val = 0.02;
margin_bottom = 0.08;
margin_top = 0.06;
avail_h = 1 - margin_bottom - margin_top - (num_links_all-1)*gap_val;
row_h = avail_h / num_links_all;

for k = 1:num_links_all
    y_bottom = margin_bottom + (num_links_all-k)*(row_h + gap_val);
    subplot('Position', [0.12, y_bottom, 0.85, row_h]);
    hold on;
    stairs(t_plot_mc, attack_matrix(:, k), '-', 'Color', [0.15 0.25 0.55], 'LineWidth', 1.2);
    area(t_plot_mc, attack_matrix(:, k), 'FaceColor', [0.55 0.65 0.85], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.55);
    stairs(t_plot_mc, attack_matrix(:, k), '-', 'Color', [0.10 0.20 0.50], 'LineWidth', 1.2);
    ylim([-0.15, 1.15]);
    yticks([0, 1]);
    yticklabels({'Safe', 'DoS'});
    xlim([0, t_plot_mc(end)]);
    % 链路序号放在纵坐标左侧
    ylabel(link_labels_all{k}, 'FontSize', 12, 'FontWeight', 'bold', ...
        'FontName', 'Times New Roman', 'Rotation', 0, ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    if k < num_links_all
        set(gca, 'XTickLabel', []);
    end
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 0.5);
    grid on;
    hold off;
end
xlabel('t(s)', 'FontSize', 12, 'FontName', 'Times New Roman');

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图8：图7 Trust Factor 统一图例（1×4，居中排列）
figure(8)
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
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 12);

% ===== 导出所有图片为高清晰度 PDF =====
output_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~isfolder(output_dir)
    mkdir(output_dir);
end

fig_list = [1, 2, 3, 4, 5, 6, 7, 8];
fig_names = {'3D_Trajectory_base', 'tgo_and_Range_base', 'MultiChannel_DoS_base', 'Ay_Az_base', ...
             'Observer_Error_base', 'Lead_Angle_Sigma_base', 'Trust_Factor_Psi_base', ...
             'Trust_Factor_Psi_Legend'};
fig_sizes = [800, 600; 800, 600; 800, 600; 800, 600; ...
             800, 600; 800, 600; 800, 600; 680, 130];

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
figure(2)
subplot(321)
plot(t(1:length(x_state)),tgo,'LineWidth',2,'LineStyle','-');
xlabel("t_{go,i}(s)")
ylabel("Range(m)")
subplot(322)
plot(t(1:length(sigma)),rad2deg(sigma),'LineWidth',2,'LineStyle','-');
xlabel("t(s)")
ylabel("\sigma(deg)")
subplot(323)
plot(t(1:length(x_state)),rad2deg(x_state(:,2:5:20)),'LineWidth',2,'LineStyle','-');
xlabel("t(s)")
ylabel("\theta_L(deg)")
subplot(324)
plot(t(1:length(x_state)),rad2deg(x_state(:,3:5:20)),'LineWidth',2,'LineStyle','-');
xlabel("t(s)")
ylabel("\phi_L(deg)")
subplot(325)
plot(t(1:length(Ay)),Ay,'LineWidth',2,'LineStyle','-');
xlabel("t(s)")
ylabel("Ay(m/s^2)")
subplot(326)
plot(t(1:length(Az)),Ay,'LineWidth',2,'LineStyle','-');
xlabel("t(s)")
ylabel("Az(m/s^2)")

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制观测器状态与真实状态的对比图
figure(3)
% z_observer_log 的维度是 [时间步数, M*M*5]
% 需要提取每个导弹对自己的观测（应该等于真实状态）以及每个导弹对其他导弹的观测

% 确保时间向量长度匹配
len_plot = min(length(x_state), length(z_observer_log));
t_plot = t(1:len_plot);

% 状态变量名称
state_names = {'r (Range)', '\theta_L (Elevation)', '\psi_L (Azimuth)', '\theta', '\psi'};
state_indices = [1, 2, 3, 4, 5];  % 状态索引

% 为每个状态变量绘制对比图（按导弹分组）
% 展示每个导弹对自己的观测（应该等于真实状态）以及导弹1对其他导弹的观测
for state_idx = 1:5
    subplot(3, 2, state_idx)
    hold on;
    for j = 1:M
        % 真实状态
        real_state_idx = 5*(j-1) + state_indices(state_idx);
        plot(t_plot(1:len_plot), x_state(1:len_plot, real_state_idx), ...
            'LineWidth', 2, 'LineStyle', '-', 'DisplayName', ['True State-Missile ', num2str(j)]);

        % 每个导弹对自己（第j个导弹）的观测（应该等于真实状态）
        % z_observer_log 中，每行是 reshape(z_observer', 1, M*M*5)
        % z_observer 是 [M, M*5]，z_observer(i, 5*(j-1)+k) 表示第 i 个导弹对第 j 个导弹的第 k 个状态
        % reshape(z_observer', 1, M*M*5) 按列展开，所以 z_observer(i, col) 在位置 (col-1)*M + i
        % 对于第 j 个导弹对自己的观测，i = j，col = 5*(j-1) + state_idx
        col_idx = 5*(j-1) + state_indices(state_idx);
        obs_idx_self = (j - 1) * 25 + state_idx;  % 第 j 个导弹对自己观测的位置
        z_self = zeros(len_plot, 1);
        for k = 1:len_plot
            z_self(k) = z_observer_log(k, obs_idx_self);
        end
        plot(t_plot(1:len_plot), z_self, ...
            'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', ['Missile ', num2str(j), ' Self-Obs.']);
    end
    xlabel("t(s)")
    if state_idx == 1
        ylabel("r (m)")
    elseif state_idx == 2
        ylabel("\theta_L (rad)")
    elseif state_idx == 3
        ylabel("\psi_L (rad)")
    elseif state_idx == 4
        ylabel("\theta (rad)")
    else
        ylabel("\psi (rad)")
    end
    title(state_names{state_idx})
    legend('Location', 'best')
    grid on;
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制观测误差（观测值 - 真实值）
% 展示每个导弹对其他导弹的观测误差
figure(4)
for state_idx = 1:5
    subplot(3, 2, state_idx)
    hold on;
    for i = 1:M  % 第 i 个导弹
        for j = 1:M  % 对第 j 个导弹的观测
            if i ~= j  % 只显示对其他导弹的观测误差（对自己的观测应该为0）
                real_state_idx = 5*(j-1) + state_indices(state_idx);
                col_idx = 5*(j-1) + state_indices(state_idx);
                obs_idx = 20*(i-1)+5*(j-1)+state_idx;  % 第 i 个导弹对第 j 个导弹观测的位置
                error = zeros(len_plot, 1);
                for k = 1:len_plot
                    error(k) = z_observer_log(k, obs_idx) - x_state(k, real_state_idx);
                end
                plot(t_plot(1:len_plot), error, ...
                    'LineWidth', 1.5, 'DisplayName', ['M', num2str(i), ' of M', num2str(j)]);
            end
        end
    end
    xlabel("t(s)")
    if state_idx == 1
        ylabel("Error r (m)")
    elseif state_idx == 2
        ylabel("Error \theta_L (rad)")
    elseif state_idx == 3
        ylabel("Error \psi_L (rad)")
    elseif state_idx == 4
        ylabel("Error \theta (rad)")
    else
        ylabel("Error \psi (rad)")
    end
    title([state_names{state_idx}, ' - Obs. Error'])
    legend('Location', 'best')
    grid on;
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制DoS攻击情况
figure(5)
% 子图1：通信拓扑状态（显示哪些链路被攻击）
subplot(3, 2, 1)
% 计算每个链路被攻击的时间占比
attack_ratio_matrix = zeros(M, M);
for r = 1:M
    for c = 1:M
        if a_base(r, c) == 1
            attack_ratio_matrix(r, c) = sum(squeeze(a_log(:, r, c)) == 0) / length(t);
        end
    end
end
imagesc(attack_ratio_matrix);
colorbar;
colormap(gca, [1 1 1; 0 1 0; 1 0 0]);  % 白色=无链路，绿色=正常，红色=被攻击
caxis([0 1]);
xlabel('Missile No.')
ylabel('Missile No.')
title('Comm. Topology Attack Ratio (redder = longer attack)')
set(gca, 'XTick', 1:M, 'YTick', 1:M);

% 子图2：累计中断时间
subplot(3, 2, 2)
hold on;
for r = 1:M
    for c = 1:M
        if a_base(r, c) == 1
            downtime_vec = squeeze(dos_downtime_log(:, r, c));
            plot(t, downtime_vec, 'LineWidth', 1.5, 'DisplayName', ['链路(', num2str(r), ',', num2str(c), ')']);
        end
    end
end
xlabel('t(s)')
ylabel('Cumulative Downtime (s)')
title('DoS Attack Cumulative Downtime')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

% 子图3：DoS事件发生次数
subplot(3, 2, 3)
hold on;
for r = 1:M
    for c = 1:M
        if a_base(r, c) == 1
            event_count_vec = squeeze(dos_event_count_log(:, r, c));
            plot(t, event_count_vec, 'LineWidth', 1.5, 'DisplayName', ['链路(', num2str(r), ',', num2str(c), ')']);
        end
    end
end
xlabel('t(s)')
ylabel('DoS Event Count')
title('DoS Attack Event Count')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

% 子图4：拉普拉斯矩阵的秩（判断图连通性）
subplot(3, 2, 4)
plot(t, rank_L_log, 'LineWidth', 2);
hold on;
plot(t, (N-1)*ones(size(t)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Threshold (N-1)');
xlabel('t(s)')
ylabel('rank(L)')
title('Rank of Laplacian Matrix (connected when rank=N-1)')
legend('rank(L)', 'Threshold', 'Location', 'best')
grid on;
hold off;

% 子图5：累计网络断联时间
subplot(3, 2, 5)
cumulative_disconnect_time_vec = zeros(size(t));
cumulative_temp = 0;
for i = 1:length(t)
    if rank_L_log(i) ~= N-1
        cumulative_temp = cumulative_temp + dt;
    end
    cumulative_disconnect_time_vec(i) = cumulative_temp;
end
plot(t, cumulative_disconnect_time_vec, 'LineWidth', 2);
xlabel('t(s)')
ylabel('Cumulative Disconn. Time (s)')
title('Cumulative Network Disconnection Time \Sigma t_c')
grid on;

% 子图6：攻击时间线（显示每个时间步被攻击的链路数量）
subplot(3, 2, 6)
attacked_links_count = zeros(size(t));
for i = 1:length(a_log)
    a_current = squeeze(a_log(i, :, :));
    attacked_links_count(i) = sum(sum(a_base == 1 & a_current == 0));
end
plot(t(1:length(attacked_links_count)), attacked_links_count, 'LineWidth', 2);
xlabel('t(s)')
ylabel('No. of Attacked Links')
title('DoS Attack Timeline (attacked links per step)')
grid on;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制每个链路的DoS攻击详细情况
figure(6)
% 统计所有存在的链路
link_list = [];
for r = 1:M
    for c = 1:M
        if a_base(r, c) == 1 && r ~= c  % 排除自环，只考虑实际通信链路
            link_list = [link_list; r, c];
        end
    end
end
num_links = size(link_list, 1);

% 为每个链路创建子图
for link_idx = 1:num_links
    r = link_list(link_idx, 1);
    c = link_list(link_idx, 2);

    % 计算子图布局（每行3个）
    rows = ceil(num_links / 3);
    subplot(rows, 3, link_idx);

    % 提取该链路的攻击状态时间序列
    link_status = squeeze(a_log(:, r, c));  % 1=正常，0=被攻击
    attack_status = 1 - link_status;  % 转换为攻击状态：1=被攻击，0=正常

    % 确保时间向量长度匹配
    len_data = length(attack_status);
    t_plot = t(1:min(len_data, length(t)));

    % 绘制攻击状态（阶梯图）
    stairs(t_plot, attack_status(1:length(t_plot)), 'LineWidth', 2, 'Color', 'r');
    hold on;

    % 绘制累计中断时间（归一化到0-1范围以便在同一图中显示）
    downtime_vec = squeeze(dos_downtime_log(:, r, c));
    len_downtime = length(downtime_vec);
    t_downtime = t(1:min(len_downtime, length(t)));
    max_downtime = max(downtime_vec);
    if max_downtime > 0
        downtime_normalized = downtime_vec(1:length(t_downtime)) / max_downtime;
        plot(t_downtime, downtime_normalized, 'LineWidth', 1.5, 'Color', 'b', 'LineStyle', '--', 'DisplayName', 'Cum. Downtime (norm.)');
    end

    % 绘制DoS事件发生次数（归一化）
    event_count_vec = squeeze(dos_event_count_log(:, r, c));
    len_events = length(event_count_vec);
    t_events = t(1:min(len_events, length(t)));
    max_events = max(event_count_vec);
    if max_events > 0
        event_normalized = event_count_vec(1:length(t_events)) / max_events;
        plot(t_events, event_normalized, 'LineWidth', 1.5, 'Color', 'g', 'LineStyle', ':', 'DisplayName', 'Event Count (norm.)');
    end

    xlabel('t(s)')
    ylabel('Status / Norm. Value')
    title(['Link (', num2str(r), ',', num2str(c), ') - DoS Attack'])
    legend('Attack (1=DoS)', 'Cum. Downtime', 'Event Count', 'Location', 'best', 'FontSize', 7)
    grid on;
    ylim([-0.1, 1.1]);
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制每个链路的累计中断时间对比
figure(7)
hold on;
colors = lines(num_links);
for link_idx = 1:num_links
    r = link_list(link_idx, 1);
    c = link_list(link_idx, 2);
    downtime_vec = squeeze(dos_downtime_log(:, r, c));
    len_downtime = length(downtime_vec);
    t_plot = t(1:min(len_downtime, length(t)));
    plot(t_plot, downtime_vec(1:length(t_plot)), 'LineWidth', 1.5, 'Color', colors(link_idx, :), ...
        'DisplayName', ['Link (', num2str(r), ',', num2str(c), ')']);
end
xlabel('t(s)')
ylabel('Cumulative Downtime (s)')
title('DoS Attack Cumulative Downtime per Link')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制每个链路的DoS事件发生次数对比
figure(8)
hold on;
for link_idx = 1:num_links
    r = link_list(link_idx, 1);
    c = link_list(link_idx, 2);
    event_count_vec = squeeze(dos_event_count_log(:, r, c));
    len_events = length(event_count_vec);
    t_plot = t(1:min(len_events, length(t)));
    plot(t_plot, event_count_vec(1:length(t_plot)), 'LineWidth', 1.5, 'Color', colors(link_idx, :), ...
        'DisplayName', ['Link (', num2str(r), ',', num2str(c), ')']);
end
xlabel('t(s)')
ylabel('DoS Event Count')
title('DoS Attack Event Count per Link')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制权重分配情况
figure(10)
for missile_idx = 1:M
    subplot(M, 1, missile_idx)
    hold on;

    % 提取该导弹的权重数据
    F = squeeze(weights_log(:, missile_idx, 1));
    omega_2 = squeeze(weights_log(:, missile_idx, 2));
    phi = squeeze(weights_log(:, missile_idx, 3));
    psi = squeeze(weights_log(:, missile_idx, 4));

    % 绘制权重
    plot(t(1:length(omega_2)), omega_2, 'r-', 'LineWidth', 2, 'DisplayName', '\omega_{2i}');
    plot(t(1:length(phi)), phi, 'g--', 'LineWidth', 1.5, 'DisplayName', '\phi_i');
    plot(t(1:length(psi)), psi, 'm--', 'LineWidth', 1.5, 'DisplayName', '\psi_i');
    plot(t(1:length(F)), F, 'k:', 'LineWidth', 1.5, 'DisplayName', 'F_i');

    xlabel('t(s)')
    ylabel('Weight / Factor')
    title(['Missile ', num2str(missile_idx), ' Weight Allocation'])
    legend('Location', 'best', 'FontSize', 8)
    grid on;
    ylim([-0.1, 1.1]);
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制各导弹的权重对比
figure(11)
subplot(2, 1, 1)
hold on;
colors = lines(M);
for missile_idx = 1:M
    F_val = squeeze(weights_log(:, missile_idx, 1));
    plot(t(1:length(F_val)), F_val, 'LineWidth', 1.5, 'Color', colors(missile_idx, :), ...
        'DisplayName', ['Missile ', num2str(missile_idx)]);
end
xlabel('t(s)')
ylabel('F_i')
title('F_i Comparison across Missiles')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

subplot(2, 1, 2)
hold on;
for missile_idx = 1:M
    omega_2 = squeeze(weights_log(:, missile_idx, 2));
    plot(t(1:length(omega_2)), omega_2, 'LineWidth', 1.5, 'Color', colors(missile_idx, :), ...
        'DisplayName', ['Missile ', num2str(missile_idx)]);
end
xlabel('t(s)')
ylabel('\omega_{2i}')
title('\omega_{2i} Comparison across Missiles')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制观测器权重对比
figure(12)
subplot(2, 1, 1)
hold on;
colors = lines(M);
for missile_idx = 1:M
    F_obs = squeeze(observer_weights_log(:, missile_idx, 1));
    plot(t(1:length(F_obs)), F_obs, 'LineWidth', 1.5, 'Color', colors(missile_idx, :), ...
        'DisplayName', ['Missile ', num2str(missile_idx)]);
end
xlabel('t(s)')
ylabel('F_i^{obs}')
title('Observer F_i Comparison across Missiles')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

subplot(2, 1, 2)
hold on;
for missile_idx = 1:M
    omega_2_obs = squeeze(observer_weights_log(:, missile_idx, 2));
    plot(t(1:length(omega_2_obs)), omega_2_obs, 'LineWidth', 1.5, 'Color', colors(missile_idx, :), ...
        'DisplayName', ['Missile ', num2str(missile_idx)]);
end
xlabel('t(s)')
ylabel('\omega_{2i}^{obs}')
title('Observer \omega_{2i} Comparison across Missiles')
legend('Location', 'best', 'FontSize', 8)
grid on;
hold off;

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 绘制每个链路的攻击状态时间线（热图形式）
figure(9)
% 创建攻击状态矩阵：行为时间，列为链路
attack_status_matrix = zeros(length(a_log), num_links);
for link_idx = 1:num_links
    r = link_list(link_idx, 1);
    c = link_list(link_idx, 2);
    link_status = squeeze(a_log(:, r, c));
    attack_status_matrix(:, link_idx) = 1 - link_status;  % 1=被攻击，0=正常
end
% 确保时间向量长度匹配
len_attack = size(attack_status_matrix, 1);
t_attack = t(1:min(len_attack, length(t)));
imagesc(t_attack, 1:num_links, attack_status_matrix(1:length(t_attack), :)');
colorbar;
colormap(gca, [0 1 0; 1 0 0]);  % 绿色=正常，红色=被攻击
caxis([0 1]);
xlabel('t(s)')
ylabel('Link No.')
title('DoS Attack Status Timeline per Link (red = attacked, green = normal)')
% 设置y轴标签为链路名称
link_labels = cell(num_links, 1);
for link_idx = 1:num_links
    r = link_list(link_idx, 1);
    c = link_list(link_idx, 2);
    link_labels{link_idx} = ['(', num2str(r), ',', num2str(c), ')'];
end
set(gca, 'YTick', 1:num_links, 'YTickLabel', link_labels);
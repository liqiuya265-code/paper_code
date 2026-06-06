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
% 障碍物仅用于可视化参考（不参与控制）
lambda_info = 0.01; % 信息可信因子衰减参数 λ
obs = obstacles(1, 1, 1); % 保留对象接口，纯DoS下不添加任何障碍物

% 创建可视化障碍物对象（仅在绘图中显示，不参与避障控制）
obs_vis = obstacles(300, 1, 1);
obs_vis.add_spherical_obstacle([-3500, -4100, 3100], 500);  % 阻挡 M1 路径
obs_vis.add_cylindrical_obstacle([-5000, -2900, 0], 500, [0, 0, 1]);  % 阻挡 M2 路径（垂直圆柱）
obs_vis.add_spherical_obstacle([-4100, -2400, 2800], 500);  % 阻挡 M3 路径
obs_vis.add_spherical_obstacle([-3200, -3800, 2900], 500);  % 阻挡 M4 路径

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
x=[12500,-30*pi/180,50*pi/180,45*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-30*pi/180,30*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,45*pi/180,-30*pi/180];

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
        omega_2i=psi_i;  % 无避障：权重仅含信息可信因子
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
        x(5*(j-1)+1:5*(j-1)+5)=RK4(i,x(5*(j-1)+1:5*(j-1)+5)',Ay(i,j),Az(i,j),dt,Vm(j));
        X(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*cos(x(5*(j-1)+3));
        Y(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*sin(x(5*(j-1)+3));
        Z(i,j)=-x(5*(j-1)+1).*sin(x(5*(j-1)+2));
        if j==M
            x_state=[x_state;x];

            mu_observer=T/(T-t(i));
            if i==1

                [Ay_obs, Az_obs,last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, ones(1,M), 1, lambda_info, x,zeros(M, M));
            else
                [Ay_obs, Az_obs,last_psi_i_obs{i}] = compute_control_from_observer(t(i), z_observer, a_now, a_base, ...
                    Vm', N, M, T, sigma_max, alpha, beta, p, q, m, miu, v, n, obs, ones(1,M), 1, lambda_info, x,last_psi_i_obs{i-1});
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
colors = lines(M);

% 统一时间长度，避免提前命中导致绘图维度不一致
len_tgo = size(tgo, 1);
t_plot_tgo = t(1:len_tgo);
len_state = size(x_state, 1);
t_plot_state = t(1:len_state);
len_acc = size(Ay, 1);
t_plot_acc = t(1:len_acc);

% 计算每个导弹的命中时刻索引（r <= 5 即视为命中）
r_all = x_state(1:len_state, 1:5:20);
hit_idx = zeros(1, M);
for j = 1:M
    h = find(r_all(:, j) <= 5, 1, 'first');
    if isempty(h), hit_idx(j) = len_state; else, hit_idx(j) = h; end
end

% 图1：3D 轨迹
figure(1)
hold on;
for j = 1:M
    plot3(X(1:hit_idx(j), j), Y(1:hit_idx(j), j), Z(1:hit_idx(j), j), ...
        'LineWidth', 2, 'Color', colors(j,:), 'DisplayName', sprintf('M%d', j));
end
set(gca, 'XDir', 'reverse');
set(gca, 'YDir', 'reverse');
plot3(0,0,0,'Marker','o','LineWidth',2)
text(0,0,0, 'Target')

% 绘制障碍物（仅可视化参考，不参与避障控制）
obs_vis.plot_obstacles();

xlabel("X(m)")
ylabel("Y(m)")
zlabel("Z(m)")
grid on;
legend('Location', 'best');

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% 图2：四个导弹 tgo 变化图
figure(2)
hold on;
for j = 1:M
    hj = min(hit_idx(j), len_tgo);
    plot(t_plot_tgo(1:hj), tgo(1:hj, j), 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel("t(s)")
ylabel("t_{go}(s)")
grid on;

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% 图3：四个导弹与目标相对距离 r 变化图
figure(3)
hold on;
for j = 1:M
    plot(t_plot_state(1:hit_idx(j)), r_all(1:hit_idx(j), j), 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel("t(s)")
ylabel("R(m)")
grid on;

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% 图4：上图 Ay，下图 Az
figure(4)
subplot(2,1,1)
hold on;
for j = 1:M
    hj = min(hit_idx(j), len_acc);
    plot(t_plot_acc(1:hj), Ay(1:hj, j), 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel("t(s)")
ylabel("Ay(m/s^2)")
grid on;

subplot(2,1,2)
hold on;
for j = 1:M
    hj = min(hit_idx(j), len_acc);
    plot(t_plot_acc(1:hj), Az(1:hj, j), 'LineWidth', 2, 'Color', colors(j,:));
end
xlabel("t(s)")
ylabel("Az(m/s^2)")
grid on;

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

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

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% 图6：前置角（heading error）变化图
len_sigma = size(sigma, 1);
t_plot_sigma = t(1:len_sigma);
figure(6)
hold on;
sigma_deg = rad2deg(sigma(:, 1:4));
for j = 1:M
    hj = min(hit_idx(j), len_sigma);
    plot(t_plot_sigma(1:hj), sigma_deg(1:hj, j), 'LineWidth', 1.8, 'Color', colors(j,:));
end
yline(rad2deg(sigma_max), 'r--', 'LineWidth', 1.5, 'DisplayName', ['\sigma_{max}=', num2str(rad2deg(sigma_max)), '\circ']);
xlabel('Time $t$ (s)', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
ylabel('Lead Angle $\sigma$ (deg)', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('Lead Angle $\sigma$ of Four Missiles', 'FontSize', 12, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
legend('Missile 1', 'Missile 2', 'Missile 3', 'Missile 4', ...
    ['\sigma_{max}=', num2str(rad2deg(sigma_max)), '\circ'], ...
    'Location', 'best', 'FontSize', 9);
grid on;
hold off;
set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% 图7：信息可信因子 psi 变化图
len_wt = min(size(weights_log, 1), length(t));
t_plot_wt = t(1:len_wt);
figure(7)
hold on;
for midx = 1:M
    psi_vals = squeeze(weights_log(1:hit_idx(midx), midx, 4));
    plot(t_plot_wt(1:hit_idx(midx)), psi_vals, 'LineWidth', 1.8, ...
        'Color', colors(midx,:), 'DisplayName', ['Missile ', num2str(midx)]);
end
xlabel('Time $t$ (s)', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
ylabel('Trust Factor $\psi_i$', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('Information Credibility Trust Factor $\psi_i$', 'FontSize', 12, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
legend('Location', 'best', 'FontSize', 9);
ylim([-0.05, 1.05]);
grid on;
hold off;
set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

% ===== 多通道异步DoS攻击示意图 =====
% 统计所有存在的通信链路（排除自环）
link_list_all = [];
for r = 1:M
    for c = 1:M
        if a_base(r, c) == 1 && r ~= c
            link_list_all = [link_list_all; r, c]; %#ok<AGROW>
        end
    end
end
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

figure(99)
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
    ylabel(link_labels_all{k}, 'FontSize', 8, 'FontWeight', 'bold', ...
        'FontName', 'Times New Roman', 'Rotation', 0, ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    if k < num_links_all
        set(gca, 'XTickLabel', []);
    end
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 8, 'LineWidth', 0.5);
    grid on;
    hold off;
end
xlabel('Time $t$ (s)', 'FontSize', 10, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
sgtitle('Multi-channel Asynchronous DoS Attack Timeline', ...
    'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

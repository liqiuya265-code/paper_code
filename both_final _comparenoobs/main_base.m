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
% 纯DoS场景：不考虑障碍物，仅保留信息可信因子 psi
lambda_info = 0.0008; % 信息可信因子衰减参数 λ
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
T_safe = 5;  % 安全时间参数
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
cumulative_disconnect_time_log = zeros(length(t), 1);  % 记录累计断联时间

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
    if rank_L < N-1
        cumulative_disconnect_time = cumulative_disconnect_time + dt;  % 累计断联时间
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
        % 计算信息可信因子 ψ_i（带低通滤波平滑）
        [psi_i, has_connections] = information_credibility_factor(z_observer, x, a_now, j, lambda_info, last_psi_i(j), 0.3);
        if ~has_connections
            psi_i = last_psi_i(j);  % 使用最近有连接时刻的 psi_i
        else
            last_psi_i(j) = psi_i;  % 更新最近的 psi_i
        end
        phi_i = 1;  % 固定环境安全因子
        omega_2i = psi_i * phi_i;
        % 记录权重日志: F_i=0
        weights_log(i, j, :) = [0, omega_2i, phi_i, psi_i];
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
% 后处理：每个导弹到达目标后截断其数据（NaN 终止绘图）
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
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

% 统一时间长度，避免提前命中导致绘图维度不一致
actual_steps = size(x_state, 1) - 1;  % 减去初始状态行
t_end = t(actual_steps);
len_tgo = actual_steps;
t_plot_tgo = t(1:len_tgo);
len_state = size(x_state, 1);  % 含初始状态
t_plot_state = t(1:len_state);
len_acc = actual_steps;
t_plot_acc = t(1:len_acc);

% 图2：tgo 与 R 合并图（2x1 排列）
figure(2)
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

% 图4：上图 Ay，下图 Az
figure(4)
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

% 图5：观测器状态收敛图（1x4，每个子图对应一个观测导弹i，绘制5维状态误差范数）
len_obs = min(size(x_state, 1), size(z_observer_log, 1));
t_plot_obs = t(1:len_obs);
roman_labels = {'i', 'ii', 'iii', 'iv'};
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
            plot(t_plot_obs, error_norm, 'LineWidth', 2, ...
                'DisplayName', ['to Missile ', num2str(j_target)]);
        end
    end
    ylabel("||e_{state}||_2")
    % 标题移到子图下方，罗马数字编号
    text(0.5, 0.8, ['(', roman_labels{i_obs}, ') Missile ', num2str(i_obs), ' State Error Norm'], ...
        'Units', 'normalized', 'HorizontalAlignment', 'center', ...
        'FontSize', 12, 'FontName', 'Times New Roman', 'Clipping', 'off');
    if i_obs == M
        xlabel("t(s)")
        xline(10, '--', 'T=10s', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');
    end
    legend('Location', 'best')
    grid on;
    xline(10, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    hold off;
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);
% 图5 legend 单独设为 12pt
set(findall(gcf, 'Type', 'Legend'), 'FontSize', 12);

% 图6：前置角（heading error）变化图
len_sigma = actual_steps;
t_plot_sigma = t(1:len_sigma);
figure(6)
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

% 图7：信息可信因子 psi 变化图
len_wt = actual_steps;
t_plot_wt = t(1:len_wt);
figure(7)
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
xline(10, '--', 'T=10s', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 18);

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

colors_dos = lines(M);  % 与轨迹线颜色一致

for k = 1:num_links_all
    y_bottom = margin_bottom + (num_links_all-k)*(row_h + gap_val);
    subplot('Position', [0.12, y_bottom, 0.85, row_h]);
    hold on;
    stairs(t_plot_mc, attack_matrix(:, k), '-', 'Color', colors_dos(k,:), 'LineWidth', 1.5);
    ylim([-0.15, 1.15]);
    yticks([0, 1]);
    yticklabels({'Safe', 'DoS'});
    xlim([0, t_plot_mc(end)]);
    % 链路序号放在纵坐标左侧，无加粗
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
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图9：累计网络断联时间
figure(9)
plot(t(1:actual_steps), cumulative_disconnect_time_log(1:actual_steps), 'b-', 'LineWidth', 2);
xlabel('Time $t$ (s)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
ylabel('Cumulative Disconnect Time (s)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('Cumulative Network Disconnection Time $\Sigma t_c$', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
xlim([0, t_end]);
grid on;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图10：拉普拉斯矩阵秩
figure(10)
hold on;
plot(t(1:actual_steps), rank_L_log(1:actual_steps), 'b-', 'LineWidth', 2);
yline(N-1, 'r--', 'LineWidth', 1.5, 'DisplayName', ['Threshold (N-1)=', num2str(N-1)]);
xlabel('Time $t$ (s)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
ylabel('rank(L)', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('Rank of Laplacian Matrix', 'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
legend('rank(L)', ['N-1=', num2str(N-1)], 'Location', 'best');
xlim([0, t_end]);
grid on;
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% ===== 保存控制量数据用于对比 =====
save('control_effort_base.mat', 'Ay', 'Az', 't');

% ===== 导出所有图片为高清晰度 PDF =====
output_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~isfolder(output_dir)
    mkdir(output_dir);
end

fig_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
fig_names = {'3D_Trajectory_base', 'tgo_and_Range_base', 'MultiChannel_DoS_base', 'Ay_Az_base', ...
             'Observer_Error_base', 'Lead_Angle_Sigma_base', 'Trust_Factor_Psi_base', ...
             'Trust_Factor_Psi_Legend', 'Cumulative_Disconnect_Time_base', 'Rank_Laplacian_base'};
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
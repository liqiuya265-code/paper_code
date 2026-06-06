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
% 纯DoS场景：无观测器，psi 基于连通性比例

% 通信拓扑（未受攻击前）
a_base=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
a_now=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
% DoS 参数
zeta_ij=1*ones(M);
mu_ij=0.52*ones(M);
kappa_ij=2*ones(M);
nu_ij=5*ones(M);
attack_prob=0.5*ones(M);
dos_downtime=zeros(M);
dos_active=zeros(M);
dos_event_count=zeros(M);
dos_last_event_time=zeros(M);
t0=0;
rng(1);
load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');
x=[12500,-45*pi/180,45*pi/180,30*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-45*pi/180,45*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,30*pi/180,-30*pi/180];

x_state=x;
% DoS攻击记录从 dos_scenario.mat 加载，无需重新初始化
rank_L_log = zeros(length(t), 1);

% 初始化权重日志记录
weights_log = zeros(length(t), M, 4);  % [F_i, omega_2i, phi_i, psi_i]

% 初始化绘图数据
X = zeros(length(t), M);
Y = zeros(length(t), M);
Z = zeros(length(t), M);
Ay = zeros(length(t), M);
Az = zeros(length(t), M);
tgo = zeros(length(t), M);
sigma = zeros(length(t), M);

for i=1:length(t)
    % 从预生成的DoS场景加载当前步的通信拓扑
    a_now = squeeze(a_log(i,:,:));
    % 记录DoS攻击状态（从预加载数据读取）
    dos_downtime = squeeze(dos_downtime_log(i,:,:));
    dos_active = squeeze(dos_active_log(i,:,:));
    dos_event_count = squeeze(dos_event_count_log(i,:,:));

    for j=1:M
        sigma(i,j)=acos(cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)));
        tgo(i,j)=(x(5*(j-1)+1))*(1+((sin(sigma(i,j))^2)/(2*(2*N-1))))/Vm(j);
    end

    for j=1:M
        epsilon(i,j)=Epsilon(tgo(i,:),a_now,j);
        % 检查是否有与 j 断开的链路
        has_disconnected = false;
        for k = 1:M
            if k ~= j && a_base(j, k) == 1 && a_now(j, k) == 0
                has_disconnected = true;
                break;
            end
        end
        if sigma(i,j) > 0.001
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

        % 无观测器：psi 基于当前连通性比例
        denom = max(sum(a_base(j,:)), 1);
        psi_j = sum(a_now(j,:)) / denom;
        omega_2i = 1;

        % 记录权重日志: F_i=0 (纯DoS无避障), phi_i=1 (无障碍物)
        weights_log(i, j, :) = [0, omega_2i, 1, psi_j];

        R_ItoL = [cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)),   cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)),   -sin(x(5*(j-1)+2));
            -sin(x(5*(j-1)+3)),               cos(x(5*(j-1)+3)),                0;
            sin(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), sin(x(5*(j-1)+2))*sin(x(5*(j-1)+3)),   cos(x(5*(j-1)+2))];
        R_LtoV = [cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)),   cos(x(5*(j-1)+4))*sin(x(5*(j-1)+5)),   -sin(x(5*(j-1)+4));
            -sin(x(5*(j-1)+5)),               cos(x(5*(j-1)+5)),                0;
            sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), sin(x(5*(j-1)+4))*sin(x(5*(j-1)+5)),   cos(x(5*(j-1)+4))];
        R_VtoL = R_LtoV';
        R_LtoI = R_ItoL';

        % 基础PNG加速度（名义控制）
        r_j = x(5*(j-1)+1);
        if r_j > 0
            Ay_png = -N*Vm(j)^2*sin(x(5*(j-1)+5))/r_j - omega_2i*Aybt(i,j);
            Az_png = -N*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))/r_j - omega_2i*Azbt(i,j);

            % 名义控制输入矢量 a_N
            a_N = R_LtoI * R_VtoL * [0; Ay_png; Az_png];
            a_S=[0;0;0];
            A = a_N+a_S;

            A_V(i,j,:)=R_LtoV*R_ItoL*A;
            Ay(i,j)=A_V(i,j,2);
            Az(i,j)=A_V(i,j,3);

            x(5*(j-1)+1:5*(j-1)+5)=RK4(i,x(5*(j-1)+1:5*(j-1)+5)',Ay(i,j),Az(i,j),dt,Vm(j));
        else
            % 导弹已命中，冻结状态
            Ay(i,j)=0;
            Az(i,j)=0;
        end
        X(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*cos(x(5*(j-1)+3));
        Y(i,j)=-x(5*(j-1)+1).*cos(x(5*(j-1)+2)).*sin(x(5*(j-1)+3));
        Z(i,j)=-x(5*(j-1)+1).*sin(x(5*(j-1)+2));
    end

    % 计算拉普拉斯矩阵并判断图的连通性
    L = compute_laplacian(a_now);
    rank_L = rank(L);
    rank_L_log(i) = rank_L;

    x_state=[x_state;x];

    if all(x(1:5:20) <= 0)
        break;
    end
end

%%
% 统一时间长度：以最后一个导弹命中时刻为截止
actual_steps = size(x_state, 1) - 1;  % 减去初始状态行
t_end = t(actual_steps);

len_tgo = actual_steps;
t_plot_tgo = t(1:len_tgo);
len_state = size(x_state, 1);
t_plot_state = t(1:len_state);
len_acc = actual_steps;
t_plot_acc = t(1:len_acc);

% 图1：三维轨迹
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
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

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
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图8：前置角（heading error）变化图
len_sigma = actual_steps;
t_plot_sigma = t(1:len_sigma);
figure(8)
hold on;
sigma_deg = rad2deg(sigma(1:len_sigma,1:4));
plot(t_plot_sigma, sigma_deg, 'LineWidth', 1.8);

xlabel(' t(s)', 'FontSize', 11, 'FontName', 'Times New Roman');
ylabel('$\sigma$ (deg)', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');


grid on;
xlim([0, t_end]);
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% 图9：信任因子 psi 变化图（基于连通性比例）
len_wt = actual_steps;
t_plot_wt = t(1:len_wt);
figure(9)
hold on;
for midx = 1:M
    psi_vals = squeeze(weights_log(1:len_wt, midx, 4));
    plot(t_plot_wt, psi_vals, 'LineWidth', 1.8, ...
        'DisplayName', ['Missile ', num2str(midx)]);
end
xlabel(' $t$ (s)', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
ylabel('Trust Factor $\psi_i$', 'FontSize', 11, 'FontName', 'Times New Roman', 'Interpreter', 'latex');

ylim([-0.05, 1.05]);
xlim([0, t_end]);
grid on;
hold off;
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% % 图5：DoS攻击分析（6子图）
% figure(5)
% % 子图1：通信拓扑攻击占比
% subplot(3, 2, 1)
% attack_ratio_matrix = zeros(M, M);
% for r = 1:M
%     for c = 1:M
%         if a_base(r, c) == 1
%             attack_ratio_matrix(r, c) = sum(squeeze(a_log(:, r, c)) == 0) / length(t);
%         end
%     end
% end
% imagesc(attack_ratio_matrix);
% colorbar;
% colormap(gca, [1 1 1; 0 1 0; 1 0 0]);
% caxis([0 1]);
% xlabel('Missile No.')
% ylabel('Missile No.')
% title('Comm. Topology Attack Ratio (redder = longer attack)')
% set(gca, 'XTick', 1:M, 'YTick', 1:M);

% % 子图2：累计中断时间
% subplot(3, 2, 2)
% hold on;
% for r = 1:M
%     for c = 1:M
%         if a_base(r, c) == 1
%             downtime_vec = squeeze(dos_downtime_log(:, r, c));
%             plot(t, downtime_vec, 'LineWidth', 1.5, 'DisplayName', ['Link (', num2str(r), ',', num2str(c), ')']);
%         end
%     end
% end
% xlabel('t(s)')
% ylabel('Cumulative Downtime (s)')
% title('DoS Attack Cumulative Downtime')
% legend('Location', 'best', 'FontSize', 8)
% grid on;
% hold off;

% % 子图3：DoS事件发生次数
% subplot(3, 2, 3)
% hold on;
% for r = 1:M
%     for c = 1:M
%         if a_base(r, c) == 1
%             event_count_vec = squeeze(dos_event_count_log(:, r, c));
%             plot(t, event_count_vec, 'LineWidth', 1.5, 'DisplayName', ['Link (', num2str(r), ',', num2str(c), ')']);
%         end
%     end
% end
% xlabel('t(s)')
% ylabel('DoS Event Count')
% title('DoS Attack Event Count')
% legend('Location', 'best', 'FontSize', 8)
% grid on;
% hold off;

% % 子图4：拉普拉斯矩阵的秩
% subplot(3, 2, 4)
% plot(t, rank_L_log, 'LineWidth', 2);
% hold on;
% plot(t, (N-1)*ones(size(t)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Threshold (N-1)');
% xlabel('t(s)')
% ylabel('rank(L)')
% title('Rank of Laplacian Matrix (connected when rank=N-1)')
% legend('rank(L)', 'Threshold', 'Location', 'best')
% grid on;
% hold off;

% % 子图5：累计网络断联时间
% subplot(3, 2, 5)
% cumulative_disconnect_time_vec = zeros(size(t));
% cumulative_temp = 0;
% for idx = 1:length(t)
%     if rank_L_log(idx) ~= N-1
%         cumulative_temp = cumulative_temp + dt;
%     end
%     cumulative_disconnect_time_vec(idx) = cumulative_temp;
% end
% plot(t, cumulative_disconnect_time_vec, 'LineWidth', 2);
% xlabel('t(s)')
% ylabel('Cumulative Disconn. Time (s)')
% title('Cumulative Network Disconnection Time \Sigma t_c')
% grid on;

% % 子图6：被攻击链路数量时间线
% subplot(3, 2, 6)
% attacked_links_count = zeros(size(t));
% for idx = 1:length(a_log)
%     a_current = squeeze(a_log(idx, :, :));
%     attacked_links_count(idx) = sum(sum(a_base == 1 & a_current == 0));
% end
% plot(t(1:length(attacked_links_count)), attacked_links_count, 'LineWidth', 2);
% xlabel('t(s)')
% ylabel('No. of Attacked Links')
% title('DoS Attack Timeline (attacked links per step)')
% grid on;

% all_txt = findall(gcf, '-property', 'FontName');
% set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% ===== 多通道异步DoS攻击示意图 =====
% 仅展示 4 条代表性链路
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

figure(3)
set(gcf, 'Position', [100, 100, 750, 500]);

gap_val = 0.02;
margin_bottom = 0.08;
margin_top = 0.06;
avail_h = 1 - margin_bottom - margin_top - (num_links_all-1)*gap_val;
row_h = avail_h / num_links_all;

colors_dos = [0.15 0.25 0.60;  0.85 0.25 0.20;  0.15 0.55 0.25;  0.80 0.45 0.10];

for k = 1:num_links_all
    y_bottom = margin_bottom + (num_links_all-k)*(row_h + gap_val);
    subplot('Position', [0.12, y_bottom, 0.85, row_h]);
    hold on;
    stairs(t_plot_mc, attack_matrix(:, k), '-', 'Color', colors_dos(k,:), 'LineWidth', 1.5);
    ylim([-0.15, 1.15]);
    yticks([0, 1]);
    yticklabels({'Safe', 'DoS'});
    xlim([0, t_plot_mc(end)]);
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
all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% % 图6：权重分配情况
% figure(6)
% for missile_idx = 1:M
%     subplot(M, 1, missile_idx)
%     hold on;
%     F = squeeze(weights_log(:, missile_idx, 1));
%     omega_2 = squeeze(weights_log(:, missile_idx, 2));
%     phi = squeeze(weights_log(:, missile_idx, 3));
%     psi = squeeze(weights_log(:, missile_idx, 4));
%     plot(t(1:length(omega_2)), omega_2, 'r-', 'LineWidth', 2, 'DisplayName', '\omega_{2i}');
%     plot(t(1:length(phi)), phi, 'g--', 'LineWidth', 1.5, 'DisplayName', '\phi_i');
%     plot(t(1:length(psi)), psi, 'm--', 'LineWidth', 1.5, 'DisplayName', '\psi_i');
%     plot(t(1:length(F)), F, 'k:', 'LineWidth', 1.5, 'DisplayName', 'F_i');
%     xlabel('t(s)')
%     ylabel('Weight / Factor')
%     title(['Missile ', num2str(missile_idx), ' Weight Allocation'])
%     legend('Location', 'best', 'FontSize', 8)
%     grid on;
%     ylim([-0.1, 1.1]);
%     hold off;
% end
% all_txt = findall(gcf, '-property', 'FontName');
% set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% % 图7：各导弹权重对比
% figure(7)
% subplot(2, 1, 1)
% hold on;
% colors = lines(M);
% for missile_idx = 1:M
%     F_val = squeeze(weights_log(:, missile_idx, 1));
%     plot(t(1:length(F_val)), F_val, 'LineWidth', 1.5, 'Color', colors(missile_idx, :), ...
%         'DisplayName', ['Missile ', num2str(missile_idx)]);
% end
% xlabel('t(s)')
% ylabel('F_i')
% title('F_i Comparison across Missiles')
% legend('Location', 'best', 'FontSize', 8)
% grid on;
% hold off;

% subplot(2, 1, 2)
% hold on;
% for missile_idx = 1:M
%     omega_2 = squeeze(weights_log(:, missile_idx, 2));
%     plot(t(1:length(omega_2)), omega_2, 'LineWidth', 1.5, 'Color', colors(missile_idx, :), ...
%         'DisplayName', ['Missile ', num2str(missile_idx)]);
% end
% xlabel('t(s)')
% ylabel('\omega_{2i}')
% title('\omega_{2i} Comparison across Missiles')
% legend('Location', 'best', 'FontSize', 8)
% grid on;
% hold off;
% all_txt = findall(gcf, '-property', 'FontName');
% set(all_txt, 'FontName', 'Times New Roman', 'FontSize', 15);

% ===== 导出所有图片为高清晰度 PDF =====
output_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~isfolder(output_dir)
    mkdir(output_dir);
end

fig_list = [1, 2, 3, 4, 8, 9];
fig_names = {'3D_Trajectory_compare', 'tgo_and_Range_compare', 'MultiChannel_DoS_compare', 'Ay_Az_compare', ...
             'Lead_Angle_Sigma_compare', 'Trust_Factor_Psi_compare'};
fig_width = 800;
fig_height = 500;

for f_idx = 1:length(fig_list)
    fig_handle = figure(fig_list(f_idx));
    set(fig_handle, 'Position', [50, 50, fig_width, fig_height]);
    set(fig_handle, 'PaperPositionMode', 'auto');

    pdf_path = fullfile(output_dir, [fig_names{f_idx}, '.pdf']);
    exportgraphics(fig_handle, pdf_path, 'Resolution', 600, 'ContentType', 'vector');
    fprintf('Exported: %s\n', pdf_path);
end
fprintf('All figures exported to %s/\n', output_dir);

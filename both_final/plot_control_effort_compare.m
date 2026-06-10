% plot_control_effort_compare.m
% 比较四种弹性因子配置下的总累积控制量（四个导弹累积加速度平方之和）
% Scenario 1: 仅信息可信因子
% Scenario 2: 仅环境安全因子
% Scenario 3: 无弹性因子
% Scenario 4: 两个弹性因子
%
% 使用前请先运行:
%   run_resilience_compare.m  （生成四种场景的 .mat 数据）

clc;
dt = 0.1;
M = 4;

% 加载四种场景的控制量数据
psi_data   = load('control_effort_psi_only.mat');
phi_data   = load('control_effort_phi_only.mat');
none_data  = load('control_effort_no_resilience.mat');
both_data  = load('control_effort_both.mat');

% 提取各场景数据
Ay_psi   = psi_data.Ay;   Az_psi   = psi_data.Az;   t_psi   = psi_data.t;
Ay_phi   = phi_data.Ay;   Az_phi   = phi_data.Az;   t_phi   = phi_data.t;
Ay_none  = none_data.Ay;  Az_none  = none_data.Az;  t_none  = none_data.t;
Ay_both  = both_data.Ay;  Az_both  = both_data.Az;  t_both  = both_data.t;

% 计算每个导弹的累积控制量，然后求和
len_psi  = size(Ay_psi,  1);
len_phi  = size(Ay_phi,  1);
len_none = size(Ay_none, 1);
len_both = size(Ay_both, 1);

cum_per_missile_psi  = compute_cumulative_effort(Ay_psi,  Az_psi,  len_psi,  M, dt);
cum_per_missile_phi  = compute_cumulative_effort(Ay_phi,  Az_phi,  len_phi,  M, dt);
cum_per_missile_none = compute_cumulative_effort(Ay_none, Az_none, len_none, M, dt);
cum_per_missile_both = compute_cumulative_effort(Ay_both, Az_both, len_both, M, dt);

% 四个导弹求和得到总累积控制量
total_psi  = sum(cum_per_missile_psi,  2);
total_phi  = sum(cum_per_missile_phi,  2);
total_none = sum(cum_per_missile_none, 2);
total_both = sum(cum_per_missile_both, 2);

t_end = max([t_psi(len_psi), t_phi(len_phi), t_none(len_none), t_both(len_both)]);

% 颜色与线型设置
psi_color   = [0.00, 0.45, 0.74];   % 蓝色 - psi only
phi_color   = [0.85, 0.33, 0.10];   % 橙色 - phi only
none_color  = [0.93, 0.69, 0.13];   % 黄色 - none
both_color  = [0.49, 0.18, 0.56];   % 紫色 - both

style_psi   = '-.';   % 点划线
style_phi   = ':';    % 点线
style_none  = '--';   % 虚线
style_both  = '-';    % 实线

% ===== 图21：总累积控制量对比（四条曲线，单图）=====
figure(21)
set(gcf, 'Position', [100, 100, 750, 550]);
hold on;

h1 = plot(t_psi(1:len_psi),   total_psi,   style_psi,  'LineWidth', 2.5, 'Color', psi_color);
h2 = plot(t_phi(1:len_phi),   total_phi,   style_phi,  'LineWidth', 2.5, 'Color', phi_color);
h3 = plot(t_none(1:len_none), total_none,  style_none, 'LineWidth', 2.5, 'Color', none_color);
h4 = plot(t_both(1:len_both), total_both,  style_both, 'LineWidth', 2.5, 'Color', both_color);

xlabel('t(s)', 'FontSize', 15, 'FontName', 'Times New Roman');
ylabel('$\sum_{i=1}^{4} \int_{0}^{t} \|A_i\|^2\, \mathrm{d}\tau\ \mathrm{(m^2/s^3)}$', ...
    'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');

xlim([0, 43]);
grid on;

legend([h1, h2, h3, h4], ...
    {'$\eta_1$ only (error-dependent)', ...
     '$\eta_2$ only (distance-dependent)', ...
     'No resilience', ...
     '$\eta_1 \cdot \eta_2$ (Full resilience)'}, ...
    'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex', ...
    'Location', 'northwest');

hold off;
set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman', 'FontSize', 15);

% ===== 图22：图例独立图 =====
figure(22)
set(gcf, 'Position', [200, 400, 900, 180], 'Color', 'w');
ax = axes('Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
hold on;

rectangle('Position', [0.02, 0.05, 0.96, 0.38], 'FaceColor', 'w', 'LineWidth', 0.8);

text(0.06, 0.34, 'Legend:', 'FontSize', 15, 'FontName', 'Times New Roman', ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');

lgd_y = [0.27, 0.19, 0.11, 0.04];
lgd_styles = {style_both, style_none, style_psi, style_phi};
lgd_colors = {both_color, none_color, psi_color, phi_color};
lgd_texts = {'Full resilience ($\eta_1 \cdot \eta_2$)', ...
             'No resilience', ...
             '$\eta_1$ only (error-dependent resilient trust factor)', ...
             '$\eta_2$ only (distance-dependent resilient trust factor)'};

for k = 1:4
    line([0.08, 0.18], [lgd_y(k), lgd_y(k)], 'LineWidth', 2.5, ...
        'LineStyle', lgd_styles{k}, 'Color', lgd_colors{k});
    text(0.20, lgd_y(k), lgd_texts{k}, 'FontSize', 15, 'FontName', 'Times New Roman', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Interpreter', 'latex');
end

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman', 'FontSize', 15);

% ===== 导出 PDF =====
output_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~isfolder(output_dir)
    mkdir(output_dir);
end

figure(21);
set(gcf, 'Position', [50, 50, 750, 550]);
set(gcf, 'PaperPositionMode', 'auto');
pdf_path = fullfile(output_dir, 'Control_Effort_4Way_Total.pdf');
exportgraphics(gcf, pdf_path, 'Resolution', 600, 'ContentType', 'vector');
fprintf('Exported: %s\n', pdf_path);

figure(22);
set(gcf, 'Position', [50, 50, 900, 180]);
set(gcf, 'PaperPositionMode', 'auto');
pdf_path_lgd = fullfile(output_dir, 'Control_Effort_4Way_Legend.pdf');
exportgraphics(gcf, pdf_path_lgd, 'Resolution', 600, 'ContentType', 'vector');
fprintf('Exported: %s\n', pdf_path_lgd);

fprintf('\nTotal cumulative control effort comparison figures saved to:\n  %s\n', output_dir);

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

% plot_control_effort_compare.m
% 比较两种场景下的累积控制量（累积加速度平方）
% Scenario 1 (main_base):      有观测器 + 弹性因子 (omega_2i = psi_i * phi_i)
% Scenario 2 (main_base_no_resilience): 有观测器 + 无弹性因子 (omega_2i = 1)
%
% 使用前请先运行 main_base.m 和 main_base_no_resilience.m

clc;
dt = 0.1;
M = 4;

% 加载两种场景的控制量数据
base_data = load('control_effort_base.mat');
no_res_data = load('control_effort_no_res.mat');

Ay_base = base_data.Ay;
Az_base = base_data.Az;
t_base = base_data.t;

Ay_no_res = no_res_data.Ay;
Az_no_res = no_res_data.Az;
t_no_res = no_res_data.t;

% 计算累积控制量: J_i(t) = sum_{k=1}^{n} (Ay(k,i)^2 + Az(k,i)^2) * dt
% 忽略 NaN 值（导弹命中后）
len_base = size(Ay_base, 1);
len_no_res = size(Ay_no_res, 1);

cum_effort_base = zeros(len_base, M);
cum_effort_no_res = zeros(len_no_res, M);

for j = 1:M
    for i = 1:len_base
        if ~isnan(Ay_base(i, j))
            effort = Ay_base(i, j)^2 + Az_base(i, j)^2;
            if i == 1
                cum_effort_base(i, j) = effort * dt;
            else
                cum_effort_base(i, j) = cum_effort_base(i-1, j) + effort * dt;
            end
        else
            cum_effort_base(i, j) = cum_effort_base(i-1, j);  % 命中后保持不变
        end
    end
    for i = 1:len_no_res
        if ~isnan(Ay_no_res(i, j))
            effort = Ay_no_res(i, j)^2 + Az_no_res(i, j)^2;
            if i == 1
                cum_effort_no_res(i, j) = effort * dt;
            else
                cum_effort_no_res(i, j) = cum_effort_no_res(i-1, j) + effort * dt;
            end
        else
            cum_effort_no_res(i, j) = cum_effort_no_res(i-1, j);
        end
    end
end

% 四枚导弹努力之和
total_effort_base = sum(cum_effort_base, 2);
total_effort_no_res = sum(cum_effort_no_res, 2);

% 使用实际仿真步数
actual_base = len_base;
actual_no_res = len_no_res;
t_end = max(t_base(actual_base), t_no_res(actual_no_res));

% ===== 图21：总累积控制量对比（四枚导弹之和）=====
figure(21)
set(gcf, 'Position', [100, 100, 700, 500]);
hold on;

plot(t_base(1:actual_base), total_effort_base(1:actual_base), ...
    'LineWidth', 2, 'Color', [0 0.4470 0.7410], 'LineStyle', '-');
plot(t_no_res(1:actual_no_res), total_effort_no_res(1:actual_no_res), ...
    'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '--');

xlabel('t(s)', 'FontSize', 15, 'FontName', 'Times New Roman');
ylabel('$\sum_{i=1}^{4} \int_{0}^{t} \|A_i\|^2\, \mathrm{d}\tau\ \mathrm{(m^2/s^3)}$', ...
    'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex');

xlim([0, t_end]);
legend({'With resilient factor $\eta_1$ ', ...
    'Without resilient factor'}, ...
    'FontSize', 15, 'FontName', 'Times New Roman', 'Interpreter', 'latex', ...
    'Orientation', 'horizontal', ...
    'Location', 'northoutside');
grid on;
hold off;

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman', 'FontSize', 15);

% 导出 PDF
output_dir = 'D:\guidance_learn\resilient_control调研\Dos攻击\IEEE-Transactions-LaTeX2e-templates-and-instructions (1)\Fig';
if ~isfolder(output_dir)
    mkdir(output_dir);
end

figure(21);
set(gcf, 'Position', [50, 50, 700, 500]);
set(gcf, 'PaperPositionMode', 'auto');
pdf_path = fullfile(output_dir, 'Control_Effort_Compare_Total.pdf');
exportgraphics(gcf, pdf_path, 'Resolution', 600, 'ContentType', 'vector');
fprintf('Exported: %s\n', pdf_path);

fprintf('Total cumulative control effort comparison figure saved.\n');

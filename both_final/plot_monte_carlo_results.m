% plot_monte_carlo_results.m
% 绘制蒙特卡洛仿真统计图
% 运行前需先执行 run_monte_carlo.m 生成 monte_carlo_results.mat
%
% 三个子图:
%   Fig X(a): Terminal miss distance vs DoS duty ratio
%   Fig X(b): Impact-time sync error vs DoS duty ratio
%   Fig X(c): Control effort vs DoS duty ratio
% 每种方法画均值曲线 + mean±std 阴影区域

clc;

%% ===== 加载数据 =====
data_file = 'monte_carlo_results.mat';
if ~exist(data_file, 'file')
    error('File %s not found. Please run run_monte_carlo.m first.', data_file);
end

load(data_file, 'results', 'mu_values', 'N_MC', 'methods', 'method_labels');

% 默认方法标签（如果文件中未保存）
if ~exist('method_labels', 'var') || isempty(method_labels)
    method_labels = {'Full resilience ($\eta_1\eta_2$)', ...
                     '$\psi$ only ($\eta_2$)', ...
                     'No resilience'};
end

n_methods = length(methods);
n_mu = length(mu_values);

%% ===== 提取统计数据 =====
r_miss_mean = zeros(n_mu, n_methods);
r_miss_std  = zeros(n_mu, n_methods);
e_tf_mean   = zeros(n_mu, n_methods);
e_tf_std    = zeros(n_mu, n_methods);
J_u_mean    = zeros(n_mu, n_methods);
J_u_std     = zeros(n_mu, n_methods);

valid_mu_idx = [];

for k = 1:n_mu
    if ~isempty(results{k})
        valid_mu_idx = [valid_mu_idx, k]; %#ok<AGROW>
        r_miss_mean(k, :) = results{k}.r_miss_mean;
        r_miss_std(k, :)  = results{k}.r_miss_std;
        e_tf_mean(k, :)   = results{k}.e_tf_mean;
        e_tf_std(k, :)    = results{k}.e_tf_std;
        J_u_mean(k, :)    = results{k}.J_u_mean;
        J_u_std(k, :)     = results{k}.J_u_std;
    end
end

mu_plot = mu_values(valid_mu_idx);
r_miss_mean = r_miss_mean(valid_mu_idx, :);
r_miss_std  = r_miss_std(valid_mu_idx, :);
e_tf_mean   = e_tf_mean(valid_mu_idx, :);
e_tf_std    = e_tf_std(valid_mu_idx, :);
J_u_mean    = J_u_mean(valid_mu_idx, :);
J_u_std     = J_u_std(valid_mu_idx, :);

n_valid = length(valid_mu_idx);
if n_valid == 0
    error('No valid results found.');
end

%% ===== 配色和线型 =====
colors = {[0.00, 0.45, 0.74],   % 蓝色 - Full resilience
          [0.85, 0.33, 0.10],   % 橙色 - psi only
          [0.50, 0.50, 0.50]};  % 灰色 - No resilience
line_styles = {'-', '--', '-.'};
line_width = 1.8;
alpha_shade = 0.18;

%% ===== 绘制三子图 =====
figure('Position', [50, 50, 1400, 420], 'Color', 'w');

% ----- (a) Terminal Miss Distance -----
subplot(1, 3, 1);
hold on;
h_leg = zeros(1, n_methods);
for m = 1:n_methods
    x_fill = [mu_plot, fliplr(mu_plot)];
    y_fill = [(r_miss_mean(:, m)' + r_miss_std(:, m)'), ...
              fliplr(r_miss_mean(:, m)' - r_miss_std(:, m)')];
    fill(x_fill, y_fill, colors{m}, 'FaceAlpha', alpha_shade, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
    h_leg(m) = plot(mu_plot, r_miss_mean(:, m), line_styles{m}, ...
        'Color', colors{m}, 'LineWidth', line_width);
end
hold off;
xlabel('DoS duty ratio $\mu$', 'FontSize', 13, 'FontName', 'Times New Roman', ...
    'Interpreter', 'latex');
ylabel('$\bar{r}_{\mathrm{miss}}$ (m)', 'FontSize', 13, ...
    'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('(a) Terminal miss distance', 'FontSize', 14, ...
    'FontName', 'Times New Roman');
legend(h_leg, method_labels, 'FontSize', 10, 'FontName', 'Times New Roman', ...
    'Interpreter', 'latex', 'Location', 'northwest');
xlim([mu_plot(1), mu_plot(end)]);
grid on;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% ----- (b) Impact-Time Sync Error -----
subplot(1, 3, 2);
hold on;
for m = 1:n_methods
    x_fill = [mu_plot, fliplr(mu_plot)];
    y_fill = [(e_tf_mean(:, m)' + e_tf_std(:, m)'), ...
              fliplr(e_tf_mean(:, m)' - e_tf_std(:, m)')];
    fill(x_fill, y_fill, colors{m}, 'FaceAlpha', alpha_shade, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(mu_plot, e_tf_mean(:, m), line_styles{m}, ...
        'Color', colors{m}, 'LineWidth', line_width);
end
hold off;
xlabel('DoS duty ratio $\mu$', 'FontSize', 13, 'FontName', 'Times New Roman', ...
    'Interpreter', 'latex');
ylabel('$\bar{e}_{t_f}$ (s)', 'FontSize', 13, ...
    'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('(b) Impact-time synchronization error', 'FontSize', 14, ...
    'FontName', 'Times New Roman');
xlim([mu_plot(1), mu_plot(end)]);
grid on;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

% ----- (c) Control Effort -----
subplot(1, 3, 3);
hold on;
for m = 1:n_methods
    x_fill = [mu_plot, fliplr(mu_plot)];
    y_fill = [(J_u_mean(:, m)' + J_u_std(:, m)'), ...
              fliplr(J_u_mean(:, m)' - J_u_std(:, m)')];
    fill(x_fill, y_fill, colors{m}, 'FaceAlpha', alpha_shade, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(mu_plot, J_u_mean(:, m), line_styles{m}, ...
        'Color', colors{m}, 'LineWidth', line_width);
end
hold off;
xlabel('DoS duty ratio $\mu$', 'FontSize', 13, 'FontName', 'Times New Roman', ...
    'Interpreter', 'latex');
ylabel('$\bar{J}_u$ (m$^2$/s$^3$)', 'FontSize', 13, ...
    'FontName', 'Times New Roman', 'Interpreter', 'latex');
title('(c) Control effort', 'FontSize', 14, ...
    'FontName', 'Times New Roman');
xlim([mu_plot(1), mu_plot(end)]);
grid on;
set(gca, 'FontName', 'Times New Roman', 'FontSize', 11);

set(findall(gcf, '-property', 'FontName'), 'FontName', 'Times New Roman');

%% ===== 导出 PDF =====
output_dir = fileparts(mfilename('fullpath'));
if isempty(output_dir), output_dir = pwd; end

% PDF
pdf_path = fullfile(output_dir, 'MonteCarlo_DoS_Results.pdf');
exportgraphics(gcf, pdf_path, 'Resolution', 600, 'ContentType', 'vector');
fprintf('PDF exported: %s\n', pdf_path);

% EPS (单独导出每个子图为 EPS，便于 LaTeX 排版)
for sp = 1:3
    figure('Position', [50, 50, 480, 400], 'Color', 'w', 'Visible', 'off');
    hold on;
    for m = 1:n_methods
        if sp == 1
            y_mean = r_miss_mean(:, m);
            y_std  = r_miss_std(:, m);
            ylab = '$\bar{r}_{\mathrm{miss}}$ (m)';
            tit = '(a) Terminal miss distance';
        elseif sp == 2
            y_mean = e_tf_mean(:, m);
            y_std  = e_tf_std(:, m);
            ylab = '$\bar{e}_{t_f}$ (s)';
            tit = '(b) Impact-time synchronization error';
        else
            y_mean = J_u_mean(:, m);
            y_std  = J_u_std(:, m);
            ylab = '$\bar{J}_u$ (m$^2$/s$^3$)';
            tit = '(c) Control effort';
        end
        x_fill = [mu_plot, fliplr(mu_plot)];
        y_fill = [(y_mean' + y_std'), fliplr(y_mean' - y_std')];
        fill(x_fill, y_fill, colors{m}, 'FaceAlpha', alpha_shade, 'EdgeColor', 'none');
        if sp == 1
            plot(mu_plot, y_mean, line_styles{m}, 'Color', colors{m}, ...
                'LineWidth', line_width, 'DisplayName', method_labels{m});
        else
            plot(mu_plot, y_mean, line_styles{m}, 'Color', colors{m}, ...
                'LineWidth', line_width, 'HandleVisibility', 'off');
        end
    end
    hold off;
    xlabel('DoS duty ratio $\mu$', 'FontSize', 14, 'FontName', 'Times New Roman', ...
        'Interpreter', 'latex');
    ylabel(ylab, 'FontSize', 14, 'FontName', 'Times New Roman', 'Interpreter', 'latex');
    if sp == 1
        legend('Location', 'northwest', 'FontSize', 11, 'FontName', 'Times New Roman', ...
            'Interpreter', 'latex');
    end
    xlim([mu_plot(1), mu_plot(end)]);
    grid on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);

    sub_labels = {'a', 'b', 'c'};
    eps_name = sprintf('MonteCarlo_DoS_%s.eps', sub_labels{sp});
    eps_path = fullfile(output_dir, eps_name);
    exportgraphics(gcf, eps_path, 'Resolution', 600, 'ContentType', 'vector');
    fprintf('EPS exported: %s\n', eps_path);
    close(gcf);
end

fprintf('\nAll figures exported successfully.\n');
fprintf('Output directory: %s\n', output_dir);

%% ===== 打印统计摘要 =====
fprintf('\n========== Monte Carlo Results Summary ==========\n');
fprintf('N_MC = %d, mu range = [%.2f, %.2f], %d values\n', ...
    N_MC, mu_plot(1), mu_plot(end), n_valid);
fprintf('%-8s %-30s %-12s %-12s %-12s\n', 'mu', 'Method', 'r_miss', 'e_tf', 'J_u');
fprintf('%-8s %-30s %-12s %-12s %-12s\n', '---', '------', '------', '----', '---');
for k = [1, round(n_valid/2), n_valid]
    for m = 1:n_methods
        fprintf('%-8.2f %-30s %6.1f±%-5.1f %6.2f±%-5.2f %8.1f±%-7.1f\n', ...
            mu_plot(k), method_labels{m}, ...
            r_miss_mean(k, m), r_miss_std(k, m), ...
            e_tf_mean(k, m), e_tf_std(k, m), ...
            J_u_mean(k, m), J_u_std(k, m));
    end
    fprintf('---\n');
end

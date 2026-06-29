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
data_file_no_obs = 'monte_carlo_results_no_obs.mat';
if exist(data_file, 'file')
    load(data_file, 'results', 'mu_values', 'N_MC', 'methods', 'method_labels', 'use_obstacles');
elseif exist(data_file_no_obs, 'file')
    data_file = data_file_no_obs;
    load(data_file, 'results', 'mu_values', 'N_MC', 'methods', 'method_labels', 'use_obstacles');
else
    error('No result file found. Please run run_monte_carlo.m first.');
end

% 推断障碍物状态
if ~exist('use_obstacles', 'var')
    use_obstacles = ~contains(data_file, 'no_obs');
end

% 默认方法标签（如果文件中未保存）
if ~exist('method_labels', 'var') || isempty(method_labels)
    method_labels = {'Resilient guidance law (89) with \eta_1 and \eta_2', ...
                     'Guidance law (84) without \eta_1 and \eta_2', ...
                     'Guidance law (84) without the distributed observer'};
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

obstacle_tag = 'With obstacles';
if exist('use_obstacles', 'var') && ~use_obstacles
    obstacle_tag = 'Without obstacles';
end

%% ===== 导出三张独立 PDF =====
output_dir = fileparts(mfilename('fullpath'));
if isempty(output_dir), output_dir = pwd; end

sub_labels = {'a', 'b', 'c'};
ylabels = {'Miss distance (m)', ...
           'Impact-time synchronization error (s)', ...
           'Control effort (m^2/s^3)'};
titles = {'(a) Terminal miss distance', ...
          '(b) Impact-time synchronization error', ...
          '(c) Control effort'};

for sp = 1:3
    figure('Position', [50, 50, 520, 400], 'Color', 'w');
    hold on;
    for m = 1:n_methods
        if sp == 1
            y_mean = r_miss_mean(:, m);
            y_std  = r_miss_std(:, m);
        elseif sp == 2
            y_mean = e_tf_mean(:, m);
            y_std  = e_tf_std(:, m);
        else
            y_mean = J_u_mean(:, m);
            y_std  = J_u_std(:, m);
        end
        x_fill = [mu_plot, fliplr(mu_plot)];
        y_fill = [(y_mean' + y_std'), fliplr(y_mean' - y_std')];
        fill(x_fill, y_fill, colors{m}, 'FaceAlpha', alpha_shade, 'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
        plot(mu_plot, y_mean, line_styles{m}, 'Color', colors{m}, ...
            'LineWidth', line_width, 'HandleVisibility', 'off');
    end
    hold off;
    xlabel('\mu', 'FontSize', 14, 'FontName', 'Times New Roman', ...
        'Interpreter', 'tex');
    ylabel(ylabels{sp}, 'FontSize', 14, 'FontName', 'Times New Roman');
    xlim([mu_plot(1), mu_plot(end)]);
    if sp == 1
        ylim([-2 10]);
    elseif sp == 3
        ylim([-2.5e5 1e6]);

    end

    grid on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);

    % 第三张图添加局部放大 (x: 0.6-0.8, y: 0-1e6)
    if sp == 3
        inset_ax = axes('Position', [0.2, 0.5, 0.35, 0.35]);
        hold(inset_ax, 'on');
        idx_zoom = mu_plot >= 0.6 & mu_plot <= 0.9;
        mu_zoom = mu_plot(idx_zoom);
        for m = 1:n_methods
            y_mean_z = J_u_mean(idx_zoom, m);
            y_std_z  = J_u_std(idx_zoom, m);
            x_fill_z = [mu_zoom, fliplr(mu_zoom)];
            y_fill_z = [(y_mean_z' + y_std_z'), fliplr(y_mean_z' - y_std_z')];
            fill(inset_ax, x_fill_z, y_fill_z, colors{m}, 'FaceAlpha', alpha_shade, ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(inset_ax, mu_zoom, y_mean_z, line_styles{m}, 'Color', colors{m}, ...
                'LineWidth', line_width, 'HandleVisibility', 'off');
        end
        hold(inset_ax, 'off');
        xlim(inset_ax, [0.6 0.9]);
        ylim(inset_ax, [2*1e4 2*1e5]);
        set(inset_ax, 'FontName', 'Times New Roman', 'FontSize', 9);
        set(inset_ax, 'XTick', [0.6 0.7 0.8]);
        box(inset_ax, 'on');
    end


    if sp == 1
        inset_ax = axes('Position', [0.2, 0.5, 0.35, 0.35]);
        hold(inset_ax, 'on');
        idx_zoom = mu_plot >= 0.6 & mu_plot <= 0.9;
        mu_zoom = mu_plot(idx_zoom);
        for m = 1:n_methods
            y_mean_z = r_miss_mean(idx_zoom, m);
            y_std_z  = r_miss_std(idx_zoom, m);
            x_fill_z = [mu_zoom, fliplr(mu_zoom)];
            y_fill_z = [(y_mean_z' + y_std_z'), fliplr(y_mean_z' - y_std_z')];
            fill(inset_ax, x_fill_z, y_fill_z, colors{m}, 'FaceAlpha', alpha_shade, ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(inset_ax, mu_zoom, y_mean_z, line_styles{m}, 'Color', colors{m}, ...
                'LineWidth', line_width, 'HandleVisibility', 'off');
        end
        hold(inset_ax, 'off');
        xlim(inset_ax, [0.6 0.9]);
        ylim(inset_ax, [-1 4]);
        set(inset_ax, 'FontName', 'Times New Roman', 'FontSize', 9);
        set(inset_ax, 'XTick', [0.6 0.7 0.8]);
        box(inset_ax, 'on');
    end

    if sp == 2
        inset_ax = axes('Position', [0.2, 0.5, 0.35, 0.35]);
        hold(inset_ax, 'on');
        idx_zoom = mu_plot >= 0.6 & mu_plot <= 0.9;
        mu_zoom = mu_plot(idx_zoom);
        for m = 1:n_methods
            y_mean_z = e_tf_mean(idx_zoom, m);
            y_std_z  = e_tf_std(idx_zoom, m);
            x_fill_z = [mu_zoom, fliplr(mu_zoom)];
            y_fill_z = [(y_mean_z' + y_std_z'), fliplr(y_mean_z' - y_std_z')];
            fill(inset_ax, x_fill_z, y_fill_z, colors{m}, 'FaceAlpha', alpha_shade, ...
                'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(inset_ax, mu_zoom, y_mean_z, line_styles{m}, 'Color', colors{m}, ...
                'LineWidth', line_width, 'HandleVisibility', 'off');
        end
        hold(inset_ax, 'off');
        xlim(inset_ax, [0.6 0.9]);
        ylim(inset_ax, [-0.5 1]);
        set(inset_ax, 'FontName', 'Times New Roman', 'FontSize', 9);
        set(inset_ax, 'XTick', [0.6 0.7 0.8]);
        box(inset_ax, 'on');
    end


    % PDF 导出
    if use_obstacles
        pdf_name = sprintf('MonteCarlo_DoS_%s.pdf', sub_labels{sp});
    else
        pdf_name = sprintf('MonteCarlo_DoS_%s_no_obs.pdf', sub_labels{sp});
    end
    pdf_path = fullfile(output_dir, pdf_name);
    exportgraphics(gcf, pdf_path, 'Resolution', 600, 'ContentType', 'vector');
    fprintf('PDF exported: %s\n', pdf_path);
end

fprintf('\nAll figures exported successfully.\n');
fprintf('Output directory: %s\n', output_dir);

%% ===== 单独导出 Legend (3×1 排列) =====
figure('Position', [100, 380, 450, 80], 'Color', 'w');
ax = axes('Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1]);
hold on;

rectangle('Position', [0.03, 0.06, 0.94, 0.88], 'FaceColor', 'w', ...
    'LineWidth', 0.8);

short_labels = {'Resilient guidance law (89) with \eta_1 and \eta_2', ...
                'Guidance law (84) without \eta_1 and \eta_2', ...
                'Guidance law (84) without the distributed observer'};

% 三行均分布局
row_y = [0.76, 0.48, 0.20];
for midx = 1:3
    cx = 0.14;
    line([cx-0.08, cx], [row_y(midx), row_y(midx)], 'LineWidth', 2.5, ...
        'Color', colors{midx}, 'LineStyle', line_styles{midx});
    text(cx+0.03, row_y(midx), short_labels{midx}, ...
        'FontSize', 12, 'FontName', 'Times New Roman', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
end

all_txt = findall(gcf, '-property', 'FontName');
set(all_txt, 'FontName', 'Times New Roman');

% 导出 legend PDF
if use_obstacles
    legend_pdf = fullfile(output_dir, 'MonteCarlo_DoS_legend.pdf');
else
    legend_pdf = fullfile(output_dir, 'MonteCarlo_DoS_legend_no_obs.pdf');
end
exportgraphics(gcf, legend_pdf, 'Resolution', 600, 'ContentType', 'vector');
fprintf('PDF exported: %s\n', legend_pdf);

%% ===== 打印统计摘要 =====
fprintf('\n========== Monte Carlo Results Summary (%s) ==========\n', obstacle_tag);
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

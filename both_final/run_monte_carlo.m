% run_monte_carlo.m
% 蒙特卡洛仿真：评估不同 DoS 占空比下三种制导律的鲁棒性
%
% 三种对比方法:
%   Method 1: 'both'   - 完全制导律 (observer + psi*phi)
%   Method 2: 'none'   - 两个弹性因子都不加 (observer + omega=1)
%   Method 3: 'no_obs' - 没有观测器 (no observer + phi only)
%
% 横轴: DoS duty ratio mu = 0.4:0.02:0.9
% 每个 mu 运行 N_MC = 100 次独立仿真
% 每次仿真随机扰动初始状态和 DoS 攻击序列

clc; clear;

%% ===== 仿真参数 =====
tf = 100;
dt = 0.1;
t = 0:dt:tf;
n_steps = length(t);

Vm_base = [300, 300, 300, 300];
N = 4;
M = 4;

sigma_max = 5*pi/180;
alpha = 5;
beta = 5;
p_param = 0.8;
q_param = 1.2;
m_param = 1.5;
miu_param = 0.3;
v_param = 0.7;
n_param = 3;
m1 = 0;
varpi = 2;

d_safe = 50;
kappa1 = 1;
kappa2 = 1;
omega_env_i_base = 2.5 * ones(1, M);
n_env = 2;
lambda_info = 0.0008;

a_base = [1,1,0,1; 1,1,1,0; 0,1,1,1; 1,0,1,1];

T = 10;

% 基准初始状态 (4导弹 x 5状态: r, theta_L, psi_L, theta, psi)
x0_base = [12500, -45*pi/180, 45*pi/180,  30*pi/180, -30*pi/180, ...
           12000, -15*pi/180, 30*pi/180,  30*pi/180,  30*pi/180, ...
           11000, -45*pi/180, 45*pi/180,  30*pi/180,  15*pi/180, ...
           11500, -30*pi/180, 50*pi/180,  30*pi/180, -30*pi/180];

%% ===== MC 参数 =====
mu_values = 0.4:0.01:0.9;     % DoS 占空比扫描
N_MC = 100;                    % 每个 mu 的蒙特卡洛次数
use_obstacles = false;          % 是否包含障碍物 (true/false)
methods = {'both', 'none', 'no_obs'};
method_labels = {'Full guidance ($\eta_1\eta_2$, observer)', ...
                 'No resilience ($\omega=1$, observer)', ...
                 'No observer ($\eta_1$ only)'};
n_methods = length(methods);
n_mu = length(mu_values);

% 初始状态扰动范围
delta_R_range = [-200, 200];        % m
delta_V_range = [-10, 10];          % m/s
delta_angle_range = [-2, 2];        % degrees (自动转为弧度)

% 输出目录和文件名（区分有无障碍物）
output_dir = fileparts(mfilename('fullpath'));
if isempty(output_dir), output_dir = pwd; end
if use_obstacles
    data_file = fullfile(output_dir, 'monte_carlo_results.mat');
else
    data_file = fullfile(output_dir, 'monte_carlo_results_no_obs.mat');
end

%% ===== 检查是否已有部分结果（支持断点续跑）=====
if exist(data_file, 'file')
    load(data_file, 'results', 'mu_values', 'N_MC', 'methods');
    fprintf('Loaded existing results from %s\n', data_file);
    start_mu_idx = find(cellfun(@(c) isempty(c) || ...
        ~isfield(c, 'r_miss_mean'), results), 1);
    if isempty(start_mu_idx)
        fprintf('All results already computed. Skipping simulation.\n');
        return;
    else
        fprintf('Resuming from mu index %d (mu = %.2f)\n', start_mu_idx, mu_values(start_mu_idx));
    end
else
    results = cell(n_mu, 1);
    start_mu_idx = 1;
end

%% ===== 检查 Parallel Computing Toolbox =====
use_parfor = false;
if license('test', 'Distrib_Computing_Toolbox')
    try
        pool = gcp('nocreate');
        if isempty(pool)
            parpool('local', min(8, feature('numcores')));
        end
        use_parfor = true;
        fprintf('Parallel computing enabled (%d workers)\n', pool.NumWorkers);
    catch
        fprintf('Parallel pool unavailable, using serial computation.\n');
    end
else
    fprintf('Parallel Computing Toolbox not available, using serial computation.\n');
end

%% ===== 主 MC 循环 =====
t_start_total = tic;

for mu_idx = start_mu_idx:n_mu
    mu_target = mu_values(mu_idx);
    T_safe = 5;  % T_safe 随 mu 增大而减小
    fprintf('\n===== mu = %.2f (%d/%d) =====\n', mu_target, mu_idx, n_mu);
    t_start_mu = tic;

    % 预分配该 mu 的结果数组
    r_miss_all = zeros(N_MC, n_methods);
    e_tf_all   = zeros(N_MC, n_methods);
    J_u_all    = zeros(N_MC, n_methods);
    hit_all    = false(N_MC, n_methods);

    % 预生成所有 MC 运行的随机种子（确保可复现）
    rng(mu_idx * 1000);
    mc_seeds = randi(2^31-1, N_MC, 1);

    if use_parfor
        % ---- 并行执行 ----
        parfor mc = 1:N_MC
            rng(mc_seeds(mc));

            % 生成 DoS 场景
            [a_log_mc, dd_log, da_log, de_log] = generate_dos_scenario_mc(...
                t, dt, a_base, mu_target, M);

            % 扰动初始状态
            [x0_mc, Vm_mc] = perturb_initial_state(x0_base, Vm_base, M, ...
                delta_R_range, delta_V_range, delta_angle_range);

            % 对三种方法分别运行
            r_miss_local = zeros(1, n_methods);
            e_tf_local   = zeros(1, n_methods);
            J_u_local    = zeros(1, n_methods);
            hit_local    = false(1, n_methods);

            for m_idx = 1:n_methods
                mode = methods{m_idx};
                [r_m, e_t, J, h] = mc_single_run(t, dt, Vm_mc, N, M, ...
                    sigma_max, alpha, beta, p_param, q_param, m_param, miu_param, ...
                    v_param, n_param, a_base, a_log_mc, dd_log, da_log, de_log, ...
                    x0_mc, T_safe, T, lambda_info, d_safe, kappa1, kappa2, ...
                    omega_env_i_base, n_env, m1, mode, use_obstacles);
                r_miss_local(m_idx) = r_m;
                e_tf_local(m_idx)   = e_t;
                J_u_local(m_idx)    = J;
                hit_local(m_idx)    = h;
            end

            r_miss_all(mc, :) = r_miss_local;
            e_tf_all(mc, :)   = e_tf_local;
            J_u_all(mc, :)    = J_u_local;
            hit_all(mc, :)    = hit_local;
        end

    else
        % ---- 串行执行 ----
        for mc = 1:N_MC
            rng(mc_seeds(mc));

            [a_log_mc, dd_log, da_log, de_log] = generate_dos_scenario_mc(...
                t, dt, a_base, mu_target, M);

            [x0_mc, Vm_mc] = perturb_initial_state(x0_base, Vm_base, M, ...
                delta_R_range, delta_V_range, delta_angle_range);

            for m_idx = 1:n_methods
                mode = methods{m_idx};
                [r_m, e_t, J, h] = mc_single_run(t, dt, Vm_mc, N, M, ...
                    sigma_max, alpha, beta, p_param, q_param, m_param, miu_param, ...
                    v_param, n_param, a_base, a_log_mc, dd_log, da_log, de_log, ...
                    x0_mc, T_safe, T, lambda_info, d_safe, kappa1, kappa2, ...
                    omega_env_i_base, n_env, m1, mode, use_obstacles);
                r_miss_all(mc, m_idx) = r_m;
                e_tf_all(mc, m_idx)   = e_t;
                J_u_all(mc, m_idx)    = J;
                hit_all(mc, m_idx)    = h;
            end

            if mod(mc, 10) == 0
                fprintf('  mu=%.2f: %d/%d completed (%.1f s elapsed)\n', ...
                    mu_target, mc, N_MC, toc(t_start_mu));
            end
        end
    end

    % 统计并保存
    res_mu = struct();
    res_mu.mu = mu_target;
    res_mu.N_MC = N_MC;
    for m_idx = 1:n_methods
        res_mu.r_miss_mean(m_idx) = mean(r_miss_all(:, m_idx));
        res_mu.r_miss_std(m_idx)  = std(r_miss_all(:, m_idx));
        res_mu.e_tf_mean(m_idx)   = mean(e_tf_all(:, m_idx));
        res_mu.e_tf_std(m_idx)    = std(e_tf_all(:, m_idx));
        res_mu.J_u_mean(m_idx)    = mean(J_u_all(:, m_idx));
        res_mu.J_u_std(m_idx)     = std(J_u_all(:, m_idx));
        res_mu.hit_rate(m_idx)    = sum(hit_all(:, m_idx)) / N_MC * 100;
        % 保存原始数据用于后续分析
        res_mu.r_miss_raw(:, m_idx) = r_miss_all(:, m_idx);
        res_mu.e_tf_raw(:, m_idx)   = e_tf_all(:, m_idx);
        res_mu.J_u_raw(:, m_idx)    = J_u_all(:, m_idx);
    end
    results{mu_idx} = res_mu;

    % 每个 mu 完成后立即保存（断点续跑保护）
    save(data_file, 'results', 'mu_values', 'N_MC', 'methods', 'method_labels', ...
        'use_obstacles', '-v7.3');

    elapsed_mu = toc(t_start_mu);
    fprintf('  mu=%.2f done in %.1f s. r_miss=[%.1f, %.1f, %.1f], e_tf=[%.2f, %.2f, %.2f], J_u=[%.1f, %.1f, %.1f]\n', ...
        mu_target, elapsed_mu, ...
        res_mu.r_miss_mean(1), res_mu.r_miss_mean(2), res_mu.r_miss_mean(3), ...
        res_mu.e_tf_mean(1), res_mu.e_tf_mean(2), res_mu.e_tf_mean(3), ...
        res_mu.J_u_mean(1), res_mu.J_u_mean(2), res_mu.J_u_mean(3));
end

fprintf('\n===== Monte Carlo simulation complete =====\n');
fprintf('Total elapsed time: %.1f min\n', toc(t_start_total)/60);
fprintf('Results saved to: %s\n', data_file);

%% ===== 辅助函数 =====

function [a_log, dos_downtime_log, dos_active_log, dos_event_count_log] = ...
    generate_dos_scenario_mc(t, dt, a_base, mu_target, M)
% 使用 Markov 链模型生成多通道异步 DoS 攻击序列
% 每个链路独立的 Markov 过程，稳态占空比 ≈ mu_target
%
% 参数:
%   t         - 时间向量
%   dt        - 时间步长
%   a_base    - 基础通信拓扑
%   mu_target - 目标 DoS 占空比 [0, 1]
%   M         - 导弹数量
%
% 返回:
%   a_log             - (n_steps x M x M) 通信拓扑日志
%   dos_downtime_log   - (n_steps x M x M) 累计中断时长
%   dos_active_log     - (n_steps x M x M) 攻击状态标记
%   dos_event_count_log - (n_steps x M x M) DoS 事件计数

n_steps = length(t);
a_log = zeros(n_steps, M, M);
dos_downtime_log = zeros(n_steps, M, M);
dos_active_log = zeros(n_steps, M, M);
dos_event_count_log = zeros(n_steps, M, M);

attack_state = zeros(M, M);
downtime = zeros(M, M);
event_count = zeros(M, M);

% Markov 链转移概率
mean_attack_dur = 0.5;  % 平均攻击持续时间 (秒)
p_off = min(dt / mean_attack_dur, 1.0);
if mu_target >= 0.99
    p_on = 1.0;  % 接近持续攻击
else
    mean_idle_dur = mean_attack_dur * (1 - mu_target) / max(mu_target, 0.001);
    p_on = min(dt / mean_idle_dur, 1.0);
end

for step = 1:n_steps
    a_now = a_base;
    for i = 1:M
        for j = 1:M
            if a_base(i, j) == 0 || i == j
                continue;
            end

            if attack_state(i, j) == 1
                if rand < p_off
                    attack_state(i, j) = 0;
                else
                    downtime(i, j) = downtime(i, j) + dt;
                end
            else
                if rand < p_on
                    attack_state(i, j) = 1;
                    downtime(i, j) = downtime(i, j) + dt;
                    event_count(i, j) = event_count(i, j) + 1;
                end
            end

            if attack_state(i, j) == 1
                a_now(i, j) = 0;
            end
        end
    end

    a_log(step, :, :) = a_now;
    dos_downtime_log(step, :, :) = downtime;
    dos_active_log(step, :, :) = attack_state;
    dos_event_count_log(step, :, :) = event_count;
end
end

function [x0_perturbed, Vm_perturbed] = perturb_initial_state(x0_base, Vm_base, M, ...
    delta_R_range, delta_V_range, delta_angle_range)
% 对初始状态施加随机扰动
% x0_base: 基准初始状态 (1 x 5*M)
% Vm_base: 基准导弹速度 (1 x M)

x0_perturbed = x0_base;
Vm_perturbed = Vm_base;

for i = 1:M
    % 距离扰动
    dR = delta_R_range(1) + (delta_R_range(2) - delta_R_range(1)) * rand;
    x0_perturbed(5*(i-1)+1) = x0_base(5*(i-1)+1) + dR;
    % 确保距离为正
    x0_perturbed(5*(i-1)+1) = max(x0_perturbed(5*(i-1)+1), 500);

    % 角度扰动 (度 -> 弧度)
    for k = 2:5
        dAng_deg = delta_angle_range(1) + (delta_angle_range(2) - delta_angle_range(1)) * rand;
        dAng_rad = dAng_deg * pi / 180;
        x0_perturbed(5*(i-1)+k) = x0_base(5*(i-1)+k) + dAng_rad;
    end

    % 速度扰动
    dV = delta_V_range(1) + (delta_V_range(2) - delta_V_range(1)) * rand;
    Vm_perturbed(i) = Vm_base(i) + dV;
    Vm_perturbed(i) = max(Vm_perturbed(i), 200);  % 最低速度 200 m/s
end
end

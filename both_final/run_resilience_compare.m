% run_resilience_compare.m
% 运行四种弹性因子配置下的仿真，保存控制量数据用于对比
% 参数与 plot_obstacle_comparison.m 完全一致
% 四种场景均启用避障:
%   1. 'both' - omega_2i = psi_i * phi_i
%   2. 'psi'  - omega_2i = psi_i
%   3. 'phi'  - omega_2i = phi_i
%   4. 'none' - omega_2i = 1

clc; clear;

%% 公共参数（与 plot_obstacle_comparison.m 一致）
tf=100; dt=0.1; t=0:dt:tf;
Vm=[300,300,300,300]; N=4; M=4;
sigma_max=5*pi/180;
alpha=5; beta=5; p=0.8; q=1.2; m=1.5; miu=0.3; v=0.7; n=3; m1=0; varpi=2;
lambda_info = 0.0008;

a_base=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];
a_now=[1,1,0,1;1,1,1,0;0,1,1,1;1,0,1,1];

zeta_ij=1*ones(M); mu_ij=0.52*ones(M);
kappa_ij=2*ones(M); nu_ij=5*ones(M); attack_prob=0.1*ones(M);
dos_downtime=zeros(M); dos_active=zeros(M);
dos_event_count=zeros(M); dos_last_event_time=zeros(M);
t0=0; rng(1);

load('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', 'dos_event_count_log');

x0=[12500,-45*pi/180,45*pi/180,30*pi/180,-30*pi/180,12000,-15*pi/180,30*pi/180,...
    30*pi/180,30*pi/180,11000,-45*pi/180,45*pi/180,30*pi/180,15*pi/180,11500,-30*pi/180,50*pi/180,30*pi/180,-30*pi/180];

T_safe = 5; T = 10;
d_safe = 50; kappa1 = 1; kappa2 = 1;
omega_env_i = 2.5 * ones(1, M); n_env = 2;

% 启用避障
use_obstacle = true;

% 四种弹性因子模式
resilience_modes = {'both', 'psi', 'phi', 'none'};
save_names = {'control_effort_both.mat', 'control_effort_psi_only.mat', ...
              'control_effort_phi_only.mat', 'control_effort_no_resilience.mat'};
mode_labels = {'both (psi*phi)', 'psi only', 'phi only', 'neither (omega=1)'};

for mode_idx = 1:length(resilience_modes)
    mode = resilience_modes{mode_idx};
    fprintf('Running resilience mode: %s ...\n', mode_labels{mode_idx});

    rng(1);  % 固定随机种子

    [~, ~, ~, ~, ~, Ay_out, Az_out, ~, ~, ~, ~] = ...
        run_single_sim(t, dt, Vm, N, M, sigma_max, alpha, beta, p, q, m, miu, v, n, ...
            a_base, a_now, a_log, dos_downtime_log, dos_active_log, dos_event_count_log, ...
            x0, T_safe, T, lambda_info, d_safe, kappa1, kappa2, omega_env_i, n_env, ...
            use_obstacle, m1, mode);

    Ay = Ay_out;
    Az = Az_out;
    save(save_names{mode_idx}, 'Ay', 'Az', 't');
    fprintf('  Saved: %s\n', save_names{mode_idx});
end

fprintf('\nAll four resilience scenarios complete.\n');
fprintf('Run plot_control_effort_compare.m to generate comparison figures.\n');

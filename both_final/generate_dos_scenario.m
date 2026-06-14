% generate_dos_scenario.m
% 预先生成 DoS 攻击场景并保存到 dos_scenario.mat
% main.m 和 main_compare.m 均加载此文件，确保两者 DoS 攻击场景完全一致

clc; clear;

tf = 100;
dt = 0.1;
t = 0:dt:tf;
M = 4;

% 通信拓扑（与 main.m / main_compare.m 保持一致）
a_base = [1,1,0,1; 1,1,1,0; 0,1,1,1; 1,0,1,1];
a_now = a_base;

% DoS 参数（与 main.m 保持一致）
zeta_ij = 1*ones(M);
mu_ij = 0.52*ones(M);%0.65
kappa_ij = 2*ones(M);
nu_ij = 5*ones(M);
% zeta_ij = 0*ones(M);
% mu_ij = 0*ones(M);
% kappa_ij = 0*ones(M);
% nu_ij = 0*ones(M);
dos_downtime = zeros(M);
dos_active = zeros(M);
dos_event_count = zeros(M);
dos_last_event_time = zeros(M);
t0 = 0;

rng(1);  % 固定随机种子

% 预分配日志数组
a_log = zeros(length(t), M, M);
dos_downtime_log = zeros(length(t), M, M);
dos_active_log = zeros(length(t), M, M);
dos_event_count_log = zeros(length(t), M, M);

% 运行 DoS 仿真（无导弹动力学，仅攻击模式）
for i = 1:length(t)
    [a_now, dos_downtime, dos_active, dos_event_count, dos_last_event_time] = ...
        apply_dos(a_now, t(i), dt, a_base, zeta_ij, mu_ij, kappa_ij, nu_ij, ...
                  dos_downtime, dos_active, dos_event_count, dos_last_event_time, t0, []);
    a_log(i,:,:) = a_now;
    dos_downtime_log(i,:,:) = dos_downtime;
    dos_active_log(i,:,:) = dos_active;
    dos_event_count_log(i,:,:) = dos_event_count;
end

% 保存到 .mat 文件
save('dos_scenario.mat', 'a_log', 'dos_downtime_log', 'dos_active_log', ...
     'dos_event_count_log', 't', 'M', 'a_base');

fprintf('DoS scenario saved to dos_scenario.mat (%d time steps)\n', length(t));

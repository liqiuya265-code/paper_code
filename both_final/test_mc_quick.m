% test_mc_quick.m
% Quick validation: 2 mu values, 3 MC runs
% Run this first to verify everything works before the full simulation.

clc; clear;

%% Minimal parameters
tf = 100; dt = 0.1; t = 0:dt:tf;
Vm_base = [300, 300, 300, 300]; N = 4; M = 4;
sigma_max = 5*pi/180; alpha = 5; beta = 5;
p_param = 0.8; q_param = 1.2; m_param = 1.5;
miu_param = 0.3; v_param = 0.7; n_param = 3; m1 = 0;
d_safe = 50; kappa1 = 1; kappa2 = 1;
omega_env_i_base = 2.5 * ones(1, M); n_env = 2;
lambda_info = 0.0008; T_safe = 5; T = 10;
a_base = [1,1,0,1; 1,1,1,0; 0,1,1,1; 1,0,1,1];
x0_base = [12500, -45*pi/180, 45*pi/180,  30*pi/180, -30*pi/180, ...
           12000, -15*pi/180, 30*pi/180,  30*pi/180,  30*pi/180, ...
           11000, -45*pi/180, 45*pi/180,  30*pi/180,  15*pi/180, ...
           11500, -30*pi/180, 50*pi/180,  30*pi/180, -30*pi/180];

mu_test = [0.4, 0.6];
N_MC_test = 3;
methods = {'both', 'psi', 'none'};
delta_R_range = [-200, 200];
delta_V_range = [-10, 10];
delta_angle_range = [-2, 2];

fprintf('=== Quick MC Test: %d mu values x %d runs x %d methods ===\n', ...
    length(mu_test), N_MC_test, length(methods));
t_total = tic;

for mi = 1:length(mu_test)
    mu_target = mu_test(mi);
    fprintf('\n--- mu = %.2f ---\n', mu_target);

    rng(mi * 100);

    for mc = 1:N_MC_test
        % Generate DoS scenario
        [a_log_mc, dd_log, da_log, de_log] = generate_dos_test(t, dt, a_base, mu_target, M);

        % Perturb initial state
        [x0_mc, Vm_mc] = perturb_test(x0_base, Vm_base, M, ...
            delta_R_range, delta_V_range, delta_angle_range);

        for m_idx = 1:length(methods)
            mode = methods{m_idx};
            [r_m, e_t, J, h] = mc_single_run(t, dt, Vm_mc, N, M, ...
                sigma_max, alpha, beta, p_param, q_param, m_param, miu_param, ...
                v_param, n_param, a_base, a_log_mc, dd_log, da_log, de_log, ...
                x0_mc, T_safe, T, lambda_info, d_safe, kappa1, kappa2, ...
                omega_env_i_base, n_env, m1, mode);
            fprintf('  mc=%d, %-6s: r_miss=%.1f, e_tf=%.2f, J_u=%.1f, hit=%d\n', ...
                mc, mode, r_m, e_t, J, h);
        end
    end
end

fprintf('\n=== Test complete in %.1f s ===\n', toc(t_total));
fprintf('If all values look reasonable, run run_monte_carlo.m for full simulation.\n');

%% Local helpers (duplicated from run_monte_carlo.m for standalone test)

function [a_log, dos_downtime_log, dos_active_log, dos_event_count_log] = ...
    generate_dos_test(t, dt, a_base, mu_target, M)
n_steps = length(t);
a_log = zeros(n_steps, M, M);
dos_downtime_log = zeros(n_steps, M, M);
dos_active_log = zeros(n_steps, M, M);
dos_event_count_log = zeros(n_steps, M, M);
attack_state = zeros(M, M);
downtime = zeros(M, M);
event_count = zeros(M, M);
mean_attack_dur = 0.5;
p_off = min(dt / mean_attack_dur, 1.0);
if mu_target >= 0.99
    p_on = 1.0;
else
    mean_idle_dur = mean_attack_dur * (1 - mu_target) / max(mu_target, 0.001);
    p_on = min(dt / mean_idle_dur, 1.0);
end
for step = 1:n_steps
    a_now = a_base;
    for i = 1:M
        for j = 1:M
            if a_base(i, j) == 0 || i == j, continue; end
            if attack_state(i, j) == 1
                if rand < p_off, attack_state(i, j) = 0;
                else, downtime(i, j) = downtime(i, j) + dt; end
            else
                if rand < p_on
                    attack_state(i, j) = 1;
                    downtime(i, j) = downtime(i, j) + dt;
                    event_count(i, j) = event_count(i, j) + 1;
                end
            end
            if attack_state(i, j) == 1, a_now(i, j) = 0; end
        end
    end
    a_log(step, :, :) = a_now;
    dos_downtime_log(step, :, :) = downtime;
    dos_active_log(step, :, :) = attack_state;
    dos_event_count_log(step, :, :) = event_count;
end
end

function [x0_p, Vm_p] = perturb_test(x0_base, Vm_base, M, dR_rng, dV_rng, dA_rng)
x0_p = x0_base; Vm_p = Vm_base;
for i = 1:M
    dR = dR_rng(1) + (dR_rng(2) - dR_rng(1)) * rand;
    x0_p(5*(i-1)+1) = max(x0_base(5*(i-1)+1) + dR, 500);
    for k = 2:5
        dA_deg = dA_rng(1) + (dA_rng(2) - dA_rng(1)) * rand;
        x0_p(5*(i-1)+k) = x0_base(5*(i-1)+k) + dA_deg * pi/180;
    end
    dV = dV_rng(1) + (dV_rng(2) - dV_rng(1)) * rand;
    Vm_p(i) = max(Vm_base(i) + dV, 200);
end
end

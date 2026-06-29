function [r_miss, e_tf, J_u, all_hit] = mc_single_run(t, dt, Vm_perturbed, N, M, ...
    sigma_max, alpha, beta, p, q, m_param, miu_param, v, n_param, ...
    a_base, a_log, dos_downtime_log, dos_active_log, dos_event_count_log, ...
    x0, T_safe, T, lambda_info, d_safe, kappa1, kappa2, omega_env_i, n_env, ...
    m1, resilience_mode, use_obstacles)
% 单次 MC 仿真，仅返回三个性能指标（省去完整日志以提高效率）
% r_miss  = max_i R_i(t_{f,i})  终端脱靶量
% e_tf    = max_i t_{f,i} - min_i t_{f,i}  时间同步误差
% J_u     = sum_i ∫ |A_i|^2 dt  控制能量
% all_hit = 是否所有导弹都命中目标 (r <= 5)

x = x0;
obs = obstacles(d_safe, kappa1, kappa2);
if use_obstacles
    obs.add_spherical_obstacle([-500, -3500, 4000], 500);
    obs.add_cylindrical_obstacle([-5000, -1800, 0], 500, [0, 0, 1]);
    obs.add_spherical_obstacle([-2000, -500, 5000], 500);
    obs.add_cylindrical_obstacle([-2000, -2800, 0], 500, [0, 0, 1]);
end

% 初始化观测器
z_observer = zeros(M, M*5);
for i = 1:M
    for j = 1:M
        if i == j
            z_observer(i, (5*(j-1)+1):5*j) = x((5*(j-1))+1:5*j);
        else
            z_observer(i, 5*(j-1)+1) = x(5*(j-1)+1) + (rand*900 + 100);
            z_observer(i, 5*(j-1)+2) = x(5*(j-1)+2) + (rand*10 - 5) * pi/180;
            z_observer(i, 5*(j-1)+3) = x(5*(j-1)+3) + (rand*10 - 5) * pi/180;
            z_observer(i, 5*(j-1)+4) = x(5*(j-1)+4) + (rand*10 - 5) * pi/180;
            z_observer(i, 5*(j-1)+5) = x(5*(j-1)+5) + (rand*10 - 5) * pi/180;
        end
    end
end

last_psi_i = zeros(M, 1);
last_psi_i_obs_prev = zeros(M, M);
omega_captured = false(1, M);
J_u = 0;
impact_time = nan(1, M);
final_r = nan(1, M);
cumulative_disconnect_time = 0;

for step = 1:length(t)
    a_now = squeeze(a_log(step,:,:));

    % tgo_matrix
    tgo_matrix = zeros(M, M);
    for i_m = 1:M
        for j_m = 1:M
            if a_base(i_m, j_m) == 1 && a_now(i_m, j_m) == 0
                r_obs_v = z_observer(i_m, 5*(j_m-1)+1);
                theta_obs_v = z_observer(i_m, 5*(j_m-1)+4);
                psi_obs_v = z_observer(i_m, 5*(j_m-1)+5);
                sigma_obs = acos(cos(theta_obs_v) * cos(psi_obs_v));
                tgo_matrix(i_m, j_m) = r_obs_v * (1 + (sin(sigma_obs)^2) / (2 * (2*N - 1))) / Vm_perturbed(j_m);
            else
                sigma_real = acos(cos(x(5*(j_m-1)+4)) * cos(x(5*(j_m-1)+5)));
                tgo_matrix(i_m, j_m) = x(5*(j_m-1)+1) * (1 + (sin(sigma_real)^2) / (2 * (2*N - 1))) / Vm_perturbed(j_m);
            end
        end
    end

    Aybt = zeros(1, M);
    Azbt = zeros(1, M);
    for j = 1:M
        if x(5*(j-1)+1) <= 5
            continue;
        end
        sigma_j = acos(cos(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)));
        tgo_j = x(5*(j-1)+1) * (1 + (sin(sigma_j)^2) / (2*(2*N-1))) / Vm_perturbed(j);
        if strcmp(resilience_mode, 'no_obs')
            epsilon_j = Epsilon(tgo_matrix(j,:), a_now, j);
        else
            epsilon_j = Epsilon(tgo_matrix(j,:), a_base, j);
        end

        if sigma_j > 0.01
            Aybt(j) = ((2*N-1) * Vm_perturbed(j)^2 * sin(x(5*(j-1)+5)) * Phi(sigma_j, sigma_max, n_param) * ...
                (alpha*sig(epsilon_j, p) + beta*sig(epsilon_j, q))) / ...
                (x(5*(j-1)+1) * tgo_j * sin(sigma_j)^2);
            Azbt(j) = ((2*N-1) * Vm_perturbed(j)^2 * sin(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)) * Phi(sigma_j, sigma_max, n_param) * ...
                (alpha*sig(epsilon_j, p) + beta*sig(epsilon_j, q))) / ...
                (x(5*(j-1)+1) * tgo_j * sin(sigma_j)^2);
        else
            Aybt(j) = ((2*N-1) * Vm_perturbed(j)^2 * sin(x(5*(j-1)+5)) * Phi(sigma_j, sigma_max, n_param) * ...
                (alpha*sig(epsilon_j, p) + beta*sig(epsilon_j, q))) / ...
                (x(5*(j-1)+1) * tgo_j);
            Azbt(j) = ((2*N-1) * Vm_perturbed(j)^2 * sin(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)) * Phi(sigma_j, sigma_max, n_param) * ...
                (alpha*sig(epsilon_j, p) + beta*sig(epsilon_j, q))) / ...
                (x(5*(j-1)+1) * tgo_j);
        end
    end
    L_mat = compute_laplacian(a_now);
    rank_L = rank(L_mat);
    if rank_L == N-1
        cumulative_disconnect_time = cumulative_disconnect_time + dt;
    end
    % if T_safe - cumulative_disconnect_time > 0
    %     kappa_observer = 1 / max(T_safe - cumulative_disconnect_time, 5);
    % else
    %     kappa_observer=2;
    % end
    kappa_observer=2;
    for j = 1:M
        % 已命中：冻结
        if x(5*(j-1)+1) <= 5
            if isnan(impact_time(j))
                impact_time(j) = t(step);
                final_r(j) = x(5*(j-1)+1);
            end
            if j == M && ~strcmp(resilience_mode, 'no_obs')
                mu_observer = T / (T - t(step));
                [Ay_obs, Az_obs, last_psi_i_obs_prev] = compute_control_from_observer(...
                    t(step), z_observer, a_now, a_base, Vm_perturbed, N, M, T, sigma_max, ...
                    alpha, beta, p, q, m_param, miu_param, v, n_param, obs, omega_env_i, n_env, ...
                    lambda_info, x, last_psi_i_obs_prev, resilience_mode);
                z_observer = observer_RK4(t(step), z_observer, a_now, kappa_observer, mu_observer, m1, ...
                    Ay_obs, Az_obs, dt, Vm_perturbed, T);
                for i_obs = 1:M
                    z_observer(i_obs, 5*(i_obs-1)+1:5*i_obs) = x(5*(i_obs-1)+1:5*i_obs);
                end
            end
            continue;
        end

        p_i = [-x(5*(j-1)+1)*cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), ...
            -x(5*(j-1)+1)*cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), ...
            -x(5*(j-1)+1)*sin(x(5*(j-1)+2))];
        R_ItoL = [cos(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), cos(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), -sin(x(5*(j-1)+2));
            -sin(x(5*(j-1)+3)), cos(x(5*(j-1)+3)), 0;
            sin(x(5*(j-1)+2))*cos(x(5*(j-1)+3)), sin(x(5*(j-1)+2))*sin(x(5*(j-1)+3)), cos(x(5*(j-1)+2))];
        R_LtoV = [cos(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), cos(x(5*(j-1)+4))*sin(x(5*(j-1)+5)), -sin(x(5*(j-1)+4));
            -sin(x(5*(j-1)+5)), cos(x(5*(j-1)+5)), 0;
            sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5)), sin(x(5*(j-1)+4))*sin(x(5*(j-1)+5)), cos(x(5*(j-1)+4))];
        R_VtoL = R_LtoV';
        R_LtoI = R_ItoL';
        v_i = R_LtoI * R_VtoL * [Vm_perturbed(j);0;0];

        [phi_i, r_ratio] = environmental_safety_factor(obs, p_i, omega_env_i(j), n_env);

        if strcmp(resilience_mode, 'no_obs')
            psi_i = 1;
        else
            [psi_i, has_connections] = information_credibility_factor(z_observer, x, a_now, j, lambda_info, last_psi_i(j), 0.3);
            if ~has_connections
                psi_i = last_psi_i(j);
            else
                last_psi_i(j) = psi_i;
            end
        end
        switch resilience_mode
            case 'both'
                omega_2i = psi_i * phi_i;
            case 'psi'
                omega_2i = psi_i;
            case 'phi'
                omega_2i = phi_i;
            case 'none'
                omega_2i = 1;
            case 'no_obs'
                omega_2i = phi_i;   % psi=1 (no observer), phi active
            otherwise
                omega_2i = psi_i * phi_i;
        end

        Ay_png = -N*Vm_perturbed(j)^2*sin(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Aybt(j);
        Az_png = -N*Vm_perturbed(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Azbt(j);

        a_N = R_LtoI * R_VtoL * [0; Ay_png; Az_png];
        [avoidance_force, obstacle_detected] = obs.compute_obstacle_avoidance(p_i', v_i, a_N);
        if obstacle_detected && ~omega_captured(j)
            omega_env_i(j) = r_ratio;
            omega_captured(j) = true;
        end
        A_ctrl = a_N + avoidance_force;
        A_V = R_LtoV * R_ItoL * A_ctrl;
        Ay_val = A_V(2);
        Az_val = A_V(3);

        % 控制量饱和 (导弹物理极限 ~20g ≈ 200 m/s²)
        Ay_val = max(min(Ay_val, 200), -200);
        Az_val = max(min(Az_val, 200), -200);

        % 累积控制能量
        J_u = J_u + (Ay_val^2 + Az_val^2) * dt;

        % RK4 积分
        x(5*(j-1)+1:5*(j-1)+5) = RK4(step, x(5*(j-1)+1:5*(j-1)+5)', Ay_val, Az_val, dt, Vm_perturbed(j));

        if x(5*(j-1)+1) <= 5 && isnan(impact_time(j))
            impact_time(j) = t(step);
            final_r(j) = x(5*(j-1)+1);
        end

        if j == M && ~strcmp(resilience_mode, 'no_obs')
            mu_observer = T / (T - t(step));

            [Ay_obs, Az_obs, last_psi_i_obs_prev] = compute_control_from_observer(...
                t(step), z_observer, a_now, a_base, Vm_perturbed, N, M, T, sigma_max, ...
                alpha, beta, p, q, m_param, miu_param, v, n_param, obs, omega_env_i, n_env, ...
                lambda_info, x, last_psi_i_obs_prev, resilience_mode);

            z_observer = observer_RK4(t(step), z_observer, a_now, kappa_observer, mu_observer, m1, ...
                Ay_obs, Az_obs, dt, Vm_perturbed, T);

            for i_obs = 1:M
                z_observer(i_obs, 5*(i_obs-1)+1:5*i_obs) = x(5*(i_obs-1)+1:5*i_obs);
            end
        end
    end

    % 所有导弹都命中则提前退出
    all_hit_now = true;
    for jj = 1:M
        if x(5*(jj-1)+1) > 5
            all_hit_now = false;
            break;
        end
    end
    if all_hit_now
        break;
    end
end

% 处理未命中的导弹（r > 5 直到仿真结束）
for j = 1:M
    if isnan(impact_time(j))
        impact_time(j) = t(end);
        final_r(j) = x(5*(j-1)+1);
    end
end

r_miss = max(max(final_r, 0));  % clip negative overshoot to zero
e_tf = max(impact_time) - min(impact_time);
all_hit = all(final_r <= 5);
end

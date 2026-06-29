function [X, Y, Z, tgo_out, sigma_out, Ay_out, Az_out, ...
    x_state_out, weights_log_out, z_observer_log_out, sim_len] = ...
    run_single_sim(t, dt, Vm, N, M, sigma_max, alpha, beta, p, q, m, miu, v, n, ...
        a_base, a_now, a_log, dos_downtime_log, dos_active_log, dos_event_count_log, ...
        x0, T_safe, T, lambda_info, d_safe, kappa1, kappa2, omega_env_i, n_env, ...
        use_obstacle, m1, resilience_mode)
% 单次仿真运行函数
% resilience_mode - 弹性因子模式: 'both'(默认), 'psi', 'phi', 'none'

if nargin < 29 || isempty(resilience_mode)
    resilience_mode = 'both';
end

x = x0;

% 预分配输出数组
n_steps = length(t);
tgo_out = zeros(n_steps, M);
sigma_out = zeros(n_steps, M);
Ay_out = zeros(n_steps, M);
Az_out = zeros(n_steps, M);
X = zeros(n_steps, M);
Y = zeros(n_steps, M);
Z = zeros(n_steps, M);

% 障碍物对象
obs = obstacles(d_safe, kappa1, kappa2);
if use_obstacle
    obs.add_cylindrical_obstacle([-3000, -4600, 0], 400, [0, 0, 1]);%([-500, -3500, 4000], 500);
    obs.add_cylindrical_obstacle([-4600, -1800, 0], 500, [0, 0, 1]);
    obs.add_cylindrical_obstacle([-3500, -3000, 0], 500, [0, 0, 1]);
    obs.add_cylindrical_obstacle([-2000, -2800, 0], 500, [0, 0, 1]);
end

% 初始化观测器
z_observer = zeros(M, M*5);
for i = 1:M
    for j = 1:M
        if i == j
            z_observer(i, (5*(j-1)+1):5*j) = x((5*(j-1))+1:5*j);
        else
            z_observer(i, 5*(j-1)+1) = x(5*(j-1)+1) + randi([100, 1000]);
            z_observer(i, 5*(j-1)+2) = x(5*(j-1)+2) + (randi([1, 10]) * pi/180);
            z_observer(i, 5*(j-1)+3) = x(5*(j-1)+3) + (randi([1, 10]) * pi/180);
            z_observer(i, 5*(j-1)+4) = x(5*(j-1)+4) + (randi([1, 10]) * pi/180);
            z_observer(i, 5*(j-1)+5) = x(5*(j-1)+5) + (randi([1, 10]) * pi/180);
        end
    end
end
z_observer_log = reshape(z_observer', 1, M*M*5);

cumulative_disconnect_time = 0;
x_state = x;
rank_L_log = zeros(length(t), 1);

% 权重日志
weights_log = zeros(length(t), M, 4);
omega_captured = false(1, M);
observer_weights_log = zeros(length(t), M, 2);

last_psi_i = zeros(M, 1);
last_psi_i_obs = cell(length(t), 1);

sim_len = length(t);
break_flag = false;

for i = 1:length(t)
    a_now = squeeze(a_log(i,:,:));
    dos_downtime = squeeze(dos_downtime_log(i,:,:));
    dos_active = squeeze(dos_active_log(i,:,:));
    dos_event_count = squeeze(dos_event_count_log(i,:,:));

    % tgo_matrix
    tgo_matrix = zeros(M, M);
    sigma_matrix = zeros(M, M);
    for i_m = 1:M
        for j_m = 1:M
            if a_base(i_m, j_m) == 1 && a_now(i_m, j_m) == 0
                r_obs_v = z_observer(i_m, 5*(j_m-1)+1);
                theta_obs_v = z_observer(i_m, 5*(j_m-1)+4);
                psi_obs_v = z_observer(i_m, 5*(j_m-1)+5);
                sigma_matrix(i_m, j_m) = acos(cos(theta_obs_v) * cos(psi_obs_v));
                tgo_matrix(i_m, j_m) = r_obs_v * (1 + (sin(sigma_matrix(i_m, j_m))^2) / (2 * (2*N - 1))) / Vm(j_m);
            else
                sigma_matrix(i_m, j_m) = acos(cos(x(5*(j_m-1)+4)) * cos(x(5*(j_m-1)+5)));
                tgo_matrix(i_m, j_m) = x(5*(j_m-1)+1) * (1 + (sin(sigma_matrix(i_m, j_m))^2) / (2 * (2*N - 1))) / Vm(j_m);
            end
        end
    end

    sigma_vec = zeros(1, M);
    tgo_vec = zeros(1, M);
    Aybt = zeros(1, M);
    Azbt = zeros(1, M);
    epsilon_vec = zeros(1, M);
    for j = 1:M
        sigma_vec(j) = acos(cos(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)));
        tgo_vec(j) = x(5*(j-1)+1) * (1 + (sin(sigma_vec(j))^2) / (2*(2*N-1))) / Vm(j);
    end

    for j = 1:M
        epsilon_vec(j) = Epsilon(tgo_matrix(j,:), a_base, j);
        if sigma_vec(j) > 0.01
            Aybt(j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+5)) * Phi(sigma_vec(j), sigma_max, n) * (alpha*sig(epsilon_vec(j), p) + beta*sig(epsilon_vec(j), q))) / ...
                (x(5*(j-1)+1) * tgo_vec(j) * sin(sigma_vec(j))^2);
            Azbt(j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)) * Phi(sigma_vec(j), sigma_max, n) * (alpha*sig(epsilon_vec(j), p) + beta*sig(epsilon_vec(j), q))) / ...
                (x(5*(j-1)+1) * tgo_vec(j) * sin(sigma_vec(j))^2);
        else
            Aybt(j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+5)) * Phi(sigma_vec(j), sigma_max, n) * (alpha*sig(epsilon_vec(j), p) + beta*sig(epsilon_vec(j), q))) / ...
                (x(5*(j-1)+1) * tgo_vec(j));
            Azbt(j) = ((2*N-1) * Vm(j)^2 * sin(x(5*(j-1)+4)) * cos(x(5*(j-1)+5)) * Phi(sigma_vec(j), sigma_max, n) * (alpha*sig(epsilon_vec(j), p) + beta*sig(epsilon_vec(j), q))) / ...
                (x(5*(j-1)+1) * tgo_vec(j));
        end
    end

    L_mat = compute_laplacian(a_now);
    rank_L = rank(L_mat);
    rank_L_log(i) = rank_L;
    if rank_L ~= N-1
        cumulative_disconnect_time = cumulative_disconnect_time + dt;
    end

    Ay_row = zeros(1, M);
    Az_row = zeros(1, M);
    X_row = zeros(1, M);
    Y_row = zeros(1, M);
    Z_row = zeros(1, M);
    obstacle_detected_row = zeros(1, M);

    for j = 1:M
        % 已命中导弹：跳过控制计算和 RK4，冻结状态
        if x(5*(j-1)+1) <= 5
            X_row(j) = -x(5*(j-1)+1) * cos(x(5*(j-1)+2)) * cos(x(5*(j-1)+3));
            Y_row(j) = -x(5*(j-1)+1) * cos(x(5*(j-1)+2)) * sin(x(5*(j-1)+3));
            Z_row(j) = -x(5*(j-1)+1) * sin(x(5*(j-1)+2));
            Ay_row(j) = 0;
            Az_row(j) = 0;
            weights_log(i, j, :) = [0, 0, 1, 1];
            if j == M
                x_state = [x_state; x];
                %kappa_observer = T_safe / max(T_safe + t(i)-cumulative_disconnect_time, 1e-6);
                kappa_observer=T / (T - t(i));
                mu_observer = T / (T - t(i));
                oe = omega_env_i; ne = n_env;
                if ~use_obstacle, oe = ones(1, M); ne = 1; end
                if i == 1
                    [Ay_obs, Az_obs, last_psi_i_obs{i}] = compute_control_from_observer(...
                        t(i), z_observer, a_now, a_base, Vm', N, M, T, sigma_max, ...
                        alpha, beta, p, q, m, miu, v, n, obs, oe, ne, lambda_info, x, zeros(M, M), resilience_mode);
                else
                    [Ay_obs, Az_obs, last_psi_i_obs{i}] = compute_control_from_observer(...
                        t(i), z_observer, a_now, a_base, Vm', N, M, T, sigma_max, ...
                        alpha, beta, p, q, m, miu, v, n, obs, oe, ne, lambda_info, x, last_psi_i_obs{i-1}, resilience_mode);
                end
                z_observer = observer_RK4(t(i), z_observer, a_now, kappa_observer, mu_observer, m1, ...
                    Ay_obs, Az_obs, dt, Vm', T);
                for i_obs = 1:M
                    z_observer(i_obs, 5*(i_obs-1)+1:5*i_obs) = x(5*(i_obs-1)+1:5*i_obs);
                end
                z_observer_log = [z_observer_log; reshape(z_observer', 1, M*M*5)];
                for i_obs = 1:M
                    [psi_i_obs, ~] = information_credibility_factor(z_observer, x, a_now, i_obs, lambda_info);
                    if use_obstacle
                        p_obs = [-z_observer(i_obs, 5*(i_obs-1)+1)*cos(z_observer(i_obs, 5*(i_obs-1)+2))*cos(z_observer(i_obs, 5*(i_obs-1)+3)), ...
                            -z_observer(i_obs, 5*(i_obs-1)+1)*cos(z_observer(i_obs, 5*(i_obs-1)+2))*sin(z_observer(i_obs, 5*(i_obs-1)+3)), ...
                            -z_observer(i_obs, 5*(i_obs-1)+1)*sin(z_observer(i_obs, 5*(i_obs-1)+2))];
                        phi_i_obs = environmental_safety_factor(obs, p_obs, omega_env_i(i_obs), n_env);
                        observer_weights_log(i, i_obs, :) = [0, psi_i_obs * phi_i_obs];
                    else
                        observer_weights_log(i, i_obs, :) = [0, psi_i_obs];
                    end
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
        v_i = R_LtoI * R_VtoL * [Vm(j);0;0];

        % 环境安全因子
        if use_obstacle
            [phi_i, r_ratio] = environmental_safety_factor(obs, p_i, omega_env_i(j), n_env);
        else
            phi_i = 1;
            r_ratio = inf;
        end

        % 信息可信因子
        [psi_i, has_connections] = information_credibility_factor(z_observer, x, a_now, j, lambda_info, last_psi_i(j), 0.3);
        if ~has_connections
            psi_i = last_psi_i(j);
        else
            last_psi_i(j) = psi_i;
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
            otherwise
                omega_2i = psi_i * phi_i;
        end

        Ay_png = -N*Vm(j)^2*sin(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Aybt(j);
        Az_png = -N*Vm(j)^2*sin(x(5*(j-1)+4))*cos(x(5*(j-1)+5))/x(5*(j-1)+1) - omega_2i*Azbt(j);

        a_N = R_LtoI * R_VtoL * [0; Ay_png; Az_png];

        % 避障控制
        if use_obstacle
            [avoidance_force, obstacle_detected] = obs.compute_obstacle_avoidance(p_i', v_i, a_N);
            F_i = obstacle_detected;
            a_S = avoidance_force;
        else
            F_i = 0;
            a_S = [0;0;0];
        end
        A_ctrl = a_N + a_S;

        weights_log(i, j, :) = [F_i, omega_2i, phi_i, psi_i];
        obstacle_detected_row(j) = F_i;

        A_V = R_LtoV * R_ItoL * A_ctrl;
        Ay_row(j) = A_V(2);
        Az_row(j) = A_V(3);

        x(5*(j-1)+1:5*(j-1)+5) = RK4(i, x(5*(j-1)+1:5*(j-1)+5)', Ay_row(j), Az_row(j), dt, Vm(j));

        X_row(j) = -x(5*(j-1)+1) * cos(x(5*(j-1)+2)) * cos(x(5*(j-1)+3));
        Y_row(j) = -x(5*(j-1)+1) * cos(x(5*(j-1)+2)) * sin(x(5*(j-1)+3));
        Z_row(j) = -x(5*(j-1)+1) * sin(x(5*(j-1)+2));

        if j == M
            x_state = [x_state; x];
            kappa_observer = 1;
            mu_observer = T / (T - t(i));

            oe = omega_env_i; ne = n_env;
            if ~use_obstacle, oe = ones(1, M); ne = 1; end
            if i == 1
                [Ay_obs, Az_obs, last_psi_i_obs{i}] = compute_control_from_observer(...
                    t(i), z_observer, a_now, a_base, Vm', N, M, T, sigma_max, ...
                    alpha, beta, p, q, m, miu, v, n, obs, oe, ne, lambda_info, x, zeros(M, M), resilience_mode);
            else
                [Ay_obs, Az_obs, last_psi_i_obs{i}] = compute_control_from_observer(...
                    t(i), z_observer, a_now, a_base, Vm', N, M, T, sigma_max, ...
                    alpha, beta, p, q, m, miu, v, n, obs, oe, ne, lambda_info, x, last_psi_i_obs{i-1}, resilience_mode);
            end

            z_observer = observer_RK4(t(i), z_observer, a_now, kappa_observer, mu_observer, m1, ...
                Ay_obs, Az_obs, dt, Vm', T);

            for i_obs = 1:M
                z_observer(i_obs, 5*(i_obs-1)+1:5*i_obs) = x(5*(i_obs-1)+1:5*i_obs);
            end
            z_observer_log = [z_observer_log; reshape(z_observer', 1, M*M*5)]; %#ok<AGROW>

            for i_obs = 1:M
                [psi_i_obs, ~] = information_credibility_factor(z_observer, x, a_now, i_obs, lambda_info);
                if use_obstacle
                    p_obs = [-z_observer(i_obs, 5*(i_obs-1)+1)*cos(z_observer(i_obs, 5*(i_obs-1)+2))*cos(z_observer(i_obs, 5*(i_obs-1)+3)), ...
                        -z_observer(i_obs, 5*(i_obs-1)+1)*cos(z_observer(i_obs, 5*(i_obs-1)+2))*sin(z_observer(i_obs, 5*(i_obs-1)+3)), ...
                        -z_observer(i_obs, 5*(i_obs-1)+1)*sin(z_observer(i_obs, 5*(i_obs-1)+2))];
                    phi_i_obs = environmental_safety_factor(obs, p_obs, omega_env_i(i_obs), n_env);
                    observer_weights_log(i, i_obs, :) = [0, psi_i_obs * phi_i_obs];
                else
                    observer_weights_log(i, i_obs, :) = [0, psi_i_obs];
                end
            end
        end

    end  % for j = 1:M

    % 检查是否所有导弹都已命中
    all_hit = all(x(1:5:5*M) <= 5);
    if all_hit
        sim_len = i;
        break_flag = true;
    end

    % 存储当前时间步数据
    tgo_out(i, :) = tgo_vec;
    sigma_out(i, :) = sigma_vec;
    Ay_out(i, :) = Ay_row;
    Az_out(i, :) = Az_row;
    X(i, :) = X_row;
    Y(i, :) = Y_row;
    Z(i, :) = Z_row;

    if break_flag
        break;
    end
end

x_state_out = x_state;
weights_log_out = weights_log;
z_observer_log_out = z_observer_log;
end

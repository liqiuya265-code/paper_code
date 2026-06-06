function [Ay_obs, Az_obs,last_psi_i] = compute_control_from_observer(t, z_observer, a_now, a_base,Vm, N, M, T, ...
    sigma_max, alpha, beta, p, q, m, miu, v, n, obs, omega_env_i, n_env, lambda_info, x, last_psi_i)
% 基于观测状态 z_observer 计算控制输入 Ay 和 Az（使用权重分配）
% 输入：
%   t - 当前时间
%   z_observer - 观测状态 (M x M*5)，每行 z_observer(i, :) 表示第 i 个导弹对所有导弹的全局观测
%   a_now - 当前通信邻接矩阵 (M x M)
%   a_base - 基础通信拓扑 (M x M)
%   Vm - 导弹速度向量 (M x 1)
%   N - 导弹数量（用于制导律）
%   M - 导弹数量
%   T - 时间参数，t < T 时偏置项为0
%   sigma_max, alpha, beta, p, q, m, miu, v, n - 制导参数
%   obs - 障碍物对象
%   kappa_env, lambda_info, d_crit - 权重分配参数
%   x - 真实状态向量
% 输出：
%   Ay_obs - 基于观测状态计算的Ay控制输入 (M x 1)
%   Az_obs - 基于观测状态计算的Az控制输入 (M x 1)

Ay_obs = zeros(M, 1);
Az_obs = zeros(M, 1);
Aybt_obs = zeros(M, 1);
Azbt_obs = zeros(M, 1);

% 确保 Vm 是列向量
if size(Vm, 2) > size(Vm, 1)
    Vm = Vm';
end

for j = 1:M
    for k=1:M
        r_obs(j,k) = z_observer(j, 5*(k-1)+1);
        theta_L_obs(j,k) = z_observer(j, 5*(k-1)+2);
        psi_L_obs(j,k) = z_observer(j, 5*(k-1)+3);
        theta_obs(j,k) = z_observer(j, 5*(k-1)+4);
        psi_obs(j,k) = z_observer(j, 5*(k-1)+5);

        % 计算 sigma（基于观测状态）
        sigma_obs(j,k) = acos(cos(theta_obs(j,k)) * cos(psi_obs(j,k)));
        tgo_obs(j,k) = r_obs(j,k) * (1 + (sin(sigma_obs(j,k))^2) / (2 * (2*N - 1))) / Vm(k);
    end
end


% 提取观测状态
for j = 1:M
    for k=1:M
        epsilon_obs(j,k) = Epsilon(tgo_obs(j,:), a_base, k);

        if sigma_obs(j,k) > 0.01
            Aybt_obs(j,k) = ((2*N-1) * Vm(k)^2 * sin(psi_obs(j,k)) * Phi(sigma_obs(j,k), sigma_max, n )  * (alpha*sig(epsilon_obs(j,k), p) + beta*sig(epsilon_obs(j,k), q))) / ...
                (r_obs(j,k) * tgo_obs(j,k) * sin(sigma_obs(j,k))^2);
            Azbt_obs(j,k) = ((2*N-1) * Vm(k)^2 * sin(theta_obs(j,k)) * cos(psi_obs(j,k)) * Phi(sigma_obs(j,k), sigma_max, n) * ...
                 (alpha*sig(epsilon_obs(j,k), p) + beta*sig(epsilon_obs(j,k), q))) / ...
                (r_obs(j,k) * tgo_obs(j,k) * sin(sigma_obs(j,k))^2);
        else
            Aybt_obs(j,k) = ((2*N-1) * Vm(k)^2 * sin(psi_obs(j,k)) * Phi(sigma_obs(j,k), sigma_max, n) * ...
                (alpha*sig(epsilon_obs(j,k), p) + beta*sig(epsilon_obs(j,k), q))) / ...
                (r_obs(j,k) * tgo_obs(j,k));
            Azbt_obs(j,k) = ((2*N-1) * Vm(k)^2 * sin(theta_obs(j,k)) * cos(psi_obs(j,k)) * Phi(sigma_obs(j,k), sigma_max, n) * ...
               (alpha*sig(epsilon_obs(j,k), p) + beta*sig(epsilon_obs(j,k), q))) / ...
                (r_obs(j,k) * tgo_obs(j,k));
        end
    end
end

% 计算最终控制输入（基于观测状态，匹配main中的逻辑）
for j = 1:M
    for k=1:M
        % 计算第 j 个导弹的当前位置（基于观测状态）
        p_i = [-r_obs(j,k)*cos(theta_L_obs(j,k))*cos(psi_L_obs(j,k)), ...
            -r_obs(j,k)*cos(theta_L_obs(j,k))*sin(psi_L_obs(j,k)), ...
            -r_obs(j,k)*sin(theta_L_obs(j,k))];

        % 计算环境安全因子 φ_i
        phi_i = environmental_safety_factor(obs, p_i, omega_env_i(k), n_env);
        % 计算信息可信因子 ψ_i
        [psi_i, has_connections] = information_credibility_factor(z_observer, x, a_now, k, lambda_info);
        if ~has_connections
            psi_i = last_psi_i(j,k);  % 使用最近有连接时刻的 psi_i
        else
            last_psi_i(j,k) = psi_i;  % 更新最近的 psi_i
        end
        % 注意：在观测器控制计算中，我们直接使用计算结果，不使用历史值

        omega_2i = psi_i * phi_i;

        % 从第 j 个导弹对自己（第 j 个导弹）的观测中提取状态变量
        R_ItoL = [cos(theta_L_obs(j,k))*cos(psi_L_obs(j,k)),   cos(theta_L_obs(j,k))*sin(psi_L_obs(j,k)),   -sin(theta_L_obs(j,k));
            -sin(psi_L_obs(j,k)),               cos(psi_L_obs(j,k)),                0;
            sin(theta_L_obs(j,k))*cos(psi_L_obs(j,k)), sin(theta_L_obs(j,k))*sin(psi_L_obs(j,k)),   cos(theta_L_obs(j,k))];
        R_LtoV = [cos(theta_obs(j,k))*cos(psi_obs(j,k)),   cos(theta_obs(j,k))*sin(psi_obs(j,k)),   -sin(theta_obs(j,k));
            -sin(psi_obs(j,k)),               cos(psi_obs(j,k)),                0;
            sin(theta_obs(j,k))*cos(psi_obs(j,k)), sin(theta_obs(j,k))*sin(psi_obs(j,k)),   cos(theta_obs(j,k))];
        R_VtoL = R_LtoV';
        R_LtoI = R_ItoL';
        % 计算导弹速度矢量（近似）
        v_i = R_LtoI * R_VtoL * [Vm(k);0;0];

        % 基础PNG加速度（名义控制，基于观测状态）
        Ay_png_obs = -N * Vm(k)^2 * sin(psi_obs(j,k)) / r_obs(j,k) - omega_2i * Aybt_obs(j,k);
        Az_png_obs = -N * Vm(k)^2 * sin(theta_obs(j,k)) * cos(psi_obs(j,k)) / r_obs(j,k) - omega_2i * Azbt_obs(j,k);

        % 名义控制输入矢量 a_N
        a_N_obs = R_LtoI * R_VtoL * [0; Ay_png_obs; Az_png_obs];

        % 计算基于CBF的障碍物避免力 a_S
        [avoidance_force, ~] = obs.compute_obstacle_avoidance(p_i', v_i, a_N_obs);

        % 合并控制（匹配main中的逻辑）
        A_obs = a_N_obs + avoidance_force;

        % 转换回速度坐标系
        A_V_obs = R_LtoV * R_ItoL * A_obs;
        Ay_obs(j,k) = A_V_obs(2);
        Az_obs(j,k) = A_V_obs(3);
    end
end

end


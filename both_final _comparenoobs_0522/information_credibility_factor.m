function [psi_i, has_connections] = information_credibility_factor(z_observer, x_true, a_now, i, lambda)
% 计算信息可信因子 ψ_i
% 输入：
%   z_observer - 观测器状态 (M x M*5)，z_observer(i,j) 表示第i个导弹对第j个导弹的观测
%   x_true - 真实状态向量 (1 x M*5) [可选，用于调试]
%   a_now - 当前通信邻接矩阵 (M x M)
%   i - 导弹索引
%   lambda - 衰减参数 λ
% 输出：
%   psi_i - 信息可信因子
%   has_connections - 是否有通信连接的标志

M = size(a_now, 1);
E_i = 0;
connected_count = 0;

% 计算与有通信连接的导弹的观测器差值
for j = 1:M
    z_i_j = z_observer(i, 5*(j-1)+1:5*j)';
    for k = 1:M
        if a_now(i, k) > 0  % 如果第 i 个导弹与第 k 个导弹存在通信连接
            % 提取第 k 个导弹对第 j 个导弹的观测状态
            z_k_j = z_observer(k, 5*(j-1)+1:5*j)';
            % 计算观测器之间的差值（使用欧几里德距离）
            observer_diff =  a_now(i, k)*norm(z_i_j - z_k_j);
            E_i = E_i + observer_diff;
            connected_count = connected_count + 1;
        end
    end
end

% 计算 psi_i
if connected_count == 0
    psi_i = 0;  % 中间值（将在main中被替换为最近值）
    has_connections = false;
else
    psi_i = exp(-lambda * E_i);
    % psi_i = min(1, Theta_i);
    has_connections = true;
end
end
function [psi_i, has_connections, E_i] = information_credibility_factor(z_observer, x_true, a_now, i, lambda, psi_prev, alpha)
% 计算信息可信因子 ψ_i（带一阶低通滤波平滑）
% 输入：
%   z_observer - 观测器状态 (M x M*5)
%   x_true - 真实状态向量 (1 x M*5) [可选]
%   a_now - 当前通信邻接矩阵 (M x M)
%   i - 导弹索引
%   lambda - 衰减参数 λ
%   psi_prev - 上一时刻的 psi_i（用于滤波，可选，默认不使用滤波）
%   alpha - 滤波系数 (0<alpha<=1, 越小越平滑, 可选，默认 0.3)
% 输出：
%   psi_i - 信息可信因子（滤波后）
%   has_connections - 是否有通信连接的标志
%   E_i - 原始观测器差异值（未滤波）

if nargin < 7 || isempty(alpha)
    alpha = 0.3;  % 默认滤波系数
end

M = size(a_now, 1);
E_i = 0;
connected_count = 0;

% 计算与有通信连接的导弹的观测器差值
for j = 1:M
    z_i_j = z_observer(i, 5*(j-1)+1:5*j)';
    for k = 1:M
        if a_now(i, k) > 0
            z_k_j = z_observer(k, 5*(j-1)+1:5*j)';
            observer_diff = a_now(i, k) * norm(z_i_j - z_k_j);
            E_i = E_i + observer_diff;
            connected_count = connected_count + 1;
        end
    end
end

% 计算原始 psi_i (raw)
if connected_count == 0
    psi_raw = 0;
    has_connections = false;
else
    psi_raw = exp(-lambda * E_i);
    has_connections = true;
end

% 一阶低通滤波：psi_i = alpha * psi_raw + (1-alpha) * psi_prev
if nargin >= 6 && ~isempty(psi_prev) && psi_prev > 0
    psi_i = alpha * psi_raw + (1 - alpha) * psi_prev;
else
    psi_i = psi_raw;
end
end
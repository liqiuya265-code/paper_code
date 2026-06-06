function a_i = weighted_control_allocation(a_N, a_S, a_T, F_i, omega_1i, omega_2i)
% 实现加权控制分配
% 输入：
%   a_N - 名义控制 (PNG) [3x1]
%   a_S - 障碍物避免控制 [3x1]
%   a_T - 协同控制 (观测器-based) [3x1]
%   F_i - 模式选择因子 (1=避障优先，0=协同优先)
%   omega_1i - 权重 ω_{1i}
%   omega_2i - 权重 ω_{2i}
% 输出：
%   a_i - 加权后的最终控制 [3x1]

% 公式: {a_i} = {a_N} + {F_i} ω_{1i} {a_S} + [ω_{1i}(1 - {F_i}) + ω_{2i}] {a_T}
term_S = F_i * omega_1i * a_S;
term_T = (omega_1i * (1 - F_i) + omega_2i) * a_T;

a_i = a_N + term_S + term_T;
end
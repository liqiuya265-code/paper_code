function phi_i = environmental_safety_factor(obs, p_i, omega, n)
% 计算环境安全因子 φ_i
% 输入：
%   obs - 障碍物对象
%   p_i - 导弹当前位置 [x, y, z]
%   obstacle_detected - 是否检测到障碍物（对应Φ_i的判断）
%   omega - 障碍安全系数 ω > 1
%   n - 余弦函数指数
% 输出：
%   phi_i - 环境安全因子

% 新的表达式：
% φ_i(r_oi) = { 1,                                    r_oi ≥ ωR
%              { cos^n(π/2 * (r̄_oi - ω)/(1 - ω)),    r_oi < ωR
% 其中 r̄_oi = r_oi / R, R 为障碍物半径


[min_h_spherical, closest_spherical_idx] = obs.check_spherical_obstacles(p_i');
[min_h_cylindrical, closest_cylindrical_idx] = obs.check_cylindrical_obstacles(p_i');

% 找到最近的障碍物
if min_h_spherical <= min_h_cylindrical
    h_min = min_h_spherical;
    closest_idx = closest_spherical_idx;
    is_spherical = true;
else
    h_min = min_h_cylindrical;
    closest_idx = closest_cylindrical_idx;
    is_spherical = false;
end

% 获取障碍物半径 R
if is_spherical && closest_idx > 0
    R = obs.spherical_radii(closest_idx);
    p_o=obs.spherical_centers{closest_idx};
    r_actual=norm(p_i-p_o);
elseif ~is_spherical && closest_idx > 0
    R = obs.cylindrical_radii(closest_idx);
    p_o=obs.cylindrical_centers{closest_idx};
    r_actual=norm(p_i(1:2)-p_o(1:2));
else
    phi_i = 1;  % 默认安全
    return;
end

% 计算到障碍物中心的实际距离 r_actual
% 障碍函数 h = r^2 - (d_safe + R)^2



% 计算归一化距离 r̄_oi = r_oi / R
r_bar_oi = r_actual / R;

% 根据新的表达式计算 φ_i
if r_actual >= omega * R
    phi_i = 1;
else
    % 计算余弦函数参数
    cos_arg = (pi/2) * (r_bar_oi - omega) / (1 - omega);
    % 确保 cos_arg 在 [-pi/2, pi/2] 范围内以避免复数结果
    phi_i = cos(cos_arg)^n;
end
end

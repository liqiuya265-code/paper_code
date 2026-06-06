function dz = observer_update(t, z_observer, a_now, kappa, mu, m1, Ay, Az, Vm,T)
% 分布式全局状态观测器更新函数
% 实现方程：dz^i/dt = A*z^i + B*f(z^i) - kappa(t)*mu^(1+m)(t)*sum(a_ij*(z^i - z^j))
%
% 输入：
%   t - 当前时间
%   z_observer - 观测状态 (M x M*5)，每行 z_observer(i, :) 表示第 i 个导弹对所有导弹的全局观测
%   a_now - 当前通信邻接矩阵 (M x M)
%   kappa - 观测器增益函数 kappa(t)，可以是标量或函数句柄
%   mu - 观测器增益函数 mu(t)，可以是标量或函数句柄
%   m - 增益指数参数
%   Ay, Az - 控制输入 (M x 1)，每个导弹的加速度
%   Vm - 导弹速度向量 (M x 1)
% 输出：
%   dz - 观测状态导数 (M x M*5)

M = size(z_observer, 1);  % 导弹数量
dz = zeros(M, M*5);  % 初始化导数

% 计算 kappa(t) 和 mu(t)
if isa(kappa, 'function_handle')
    kappa_t = kappa(t);
else
    kappa_t = kappa;
end

if isa(mu, 'function_handle')
    mu_t = mu(t);
else
    mu_t = mu;
end

% 计算增益项：kappa(t) * mu^(1+m)(t)
%  if t<T*0.98
% gain = kappa_t * (mu_t^(1 + m1));
%  elseif t<T && t>T*0.98
%     gain=2;
%     else
%        gain=5; 
if t<T*0.9
    gain = kappa_t * (mu_t^(1 + m1));
else
       gain=2; 
 end

% 对每个导弹计算观测器导数
for i = 1:M
    % z_i 是第 i 个导弹对所有导弹的观测向量 (M*5 维)
    z_i = z_observer(i, :)';  % 第i个导弹的全局观测状态
    
    % 对每个被观测的导弹 j，计算其状态导数
    for j = 1:M
        % 提取第 i 个导弹对第 j 个导弹的观测状态（5维）
        z_i_j = z_observer(i, 5*(j-1)+1:5*j)';
        
        % 第一部分：A*z^i_j + B*f(z^i_j) - 第 j 个导弹的运动学模型
        % 使用第 j 个导弹的控制输入（因为这是对第 j 个导弹状态的观测）
        % 当 t > T 时，如果传入的是真实状态的控制输入，则观测器的运动学与真实系统一致
        dz_kinematics_j = observer_dynamics(t, z_i_j, Ay(i,j), Az(i,j), Vm(j));
        
        % 第二部分：分布式协调项
        % -kappa(t)*mu^(1+m)(t)*sum(a_ij*(z^i - z^k))，其中 z^k 是第 k 个导弹对全局的观测
        coordination_term_j = zeros(5, 1);
        for k = 1:M
            if a_now(i, k) > 0  % 如果第 i 个导弹与第 k 个导弹存在通信连接
                % 提取第 k 个导弹对第 j 个导弹的观测状态
                z_k_j = z_observer(k, 5*(j-1)+1:5*j)';
                % 协调项：z^i_j - z^k_j（第 i 个导弹和第 k 个导弹对第 j 个导弹观测的差异）
                coordination_term_j = coordination_term_j + a_now(i, k) * (z_i_j - z_k_j);
            end
        end
       
        % 完整的观测器方程（对第 j 个导弹状态的观测）
        dz(i, 5*(j-1)+1:5*j) = (dz_kinematics_j - gain * coordination_term_j)';
  
    end
end

end


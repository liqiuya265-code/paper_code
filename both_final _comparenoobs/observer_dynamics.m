function dz = observer_dynamics(t, z, Ay, Az, Vm)
% 观测器动力学函数：实现 A*z + B*f(z) 部分
% 输入：
%   t - 时间
%   z - 观测状态 [r, theta_L, psi_L, theta, psi]
%   Ay, Az - 控制输入（加速度）
%   Vm - 导弹速度
% 输出：
%   dz - 状态导数
%
% 这部分表示导弹的运动学模型：A*z + B*f(z)
% 由于原系统主要是非线性的，我们将整个运动学模型作为 B*f(z) 部分

% 调用原始的运动学模型
dz = dx_get(t, z, Ay, Az, Vm);

end





function z_new = observer_RK4(t, z_observer, a_now, kappa, mu, m1, Ay, Az, h, Vm,T)
% 使用四阶龙格-库塔方法更新观测器状态
% 输入：
%   t - 当前时间
%   z_observer - 当前观测状态 (M x 5)
%   a_now - 当前通信邻接矩阵 (M x M)
%   kappa - 观测器增益 kappa(t)
%   mu - 观测器增益 mu(t)
%   m - 增益指数参数
%   Ay, Az - 控制输入 (M x 1)
%   h - 时间步长
%   Vm - 导弹速度向量 (M x 1)
% 输出：
%   z_new - 更新后的观测状态 (M x 5)

k1 = observer_update(t, z_observer, a_now, kappa, mu, m1, Ay, Az, Vm,T);
k2 = observer_update(t + h/2, z_observer + h*k1/2, a_now, kappa, mu, m1, Ay, Az, Vm,T);
k3 = observer_update(t + h/2, z_observer + h*k2/2, a_now, kappa, mu, m1, Ay, Az, Vm,T);
k4 = observer_update(t + h, z_observer + h*k3, a_now, kappa, mu, m1, Ay, Az, Vm,T);

z_new = z_observer + h*(k1 + 2*k2 + 2*k3 + k4)/6;

end





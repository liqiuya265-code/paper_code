% 测试信息可信因子计算
clc; clear;

% 创建测试数据
M = 4;
lambda = 0.1;

% 模拟观测器状态 (M x M*5)
% z_observer(i, 5*(j-1)+1:5*j) 表示第i个导弹对第j个导弹的观测
z_observer = zeros(M, M*5);

% 设置观测值（添加一些差异来模拟观测器误差）
for i = 1:M
    for j = 1:M
        if i == j
            % 对自己的观测（假设比较准确）
            base_state = [15000-500*j, 0, 0, 0, 0];
            z_observer(i, 5*(j-1)+1:5*j) = base_state + 0.01 * randn(1,5); % 小误差
        else
            % 对其他导弹的观测（有较大误差）
            base_state = [15000-500*j, 0, 0, 0, 0];
            z_observer(i, 5*(j-1)+1:5*j) = base_state + 0.5 * randn(1,5); % 较大误差
        end
    end
end

% 测试不同的通信拓扑
fprintf('测试信息可信因子计算：\n\n');

% 测试1：完全连通
a_now_full = ones(M) - eye(M);  % 全连通，除对角线
fprintf('测试1：完全连通拓扑\n');
for i = 1:M
    psi_i = information_credibility_factor(z_observer, [], a_now_full, i, lambda);
    fprintf('  导弹 %d: ψ_i = %.4f\n', i, psi_i);
end

fprintf('\n');

% 测试2：部分连通（导弹1只与导弹2连通）
a_now_partial = zeros(M);
a_now_partial(1,2) = 1;
a_now_partial(2,1) = 1;
fprintf('测试2：部分连通拓扑（只考虑导弹1和2的连接）\n');
for i = 1:2
    psi_i = information_credibility_factor(z_observer, [], a_now_partial, i, lambda);
    fprintf('  导弹 %d: ψ_i = %.4f\n', i, psi_i);
end

fprintf('\n');

% 测试3：无连接（模拟历史值保持）
a_now_none = zeros(M);
fprintf('测试3：无连接拓扑（模拟历史值保持）\n');

% 初始化历史 psi_i
last_psi_history = 0.8 * ones(M, 1);  % 假设之前有较好的连接

for i = 1:M
    [psi_i_raw, has_connections] = information_credibility_factor(z_observer, [], a_now_none, i, lambda);
    if ~has_connections
        psi_i = last_psi_history(i);  % 使用历史值
        fprintf('  导弹 %d: ψ_i = %.4f (使用历史值 %.4f)\n', i, psi_i, last_psi_history(i));
    else
        psi_i = psi_i_raw;
        last_psi_history(i) = psi_i;  % 更新历史值
        fprintf('  导弹 %d: ψ_i = %.4f\n', i, psi_i);
    end
end

fprintf('\n');

% 测试4：从有连接变为无连接
fprintf('测试4：连接状态变化\n');
a_now_connected = zeros(M);
a_now_connected(1,2) = 1;  % 导弹1与2连接
a_now_connected(2,1) = 1;

% 首先有连接
fprintf('  有连接时：\n');
for i = 1:2
    [psi_i_raw, has_connections] = information_credibility_factor(z_observer, [], a_now_connected, i, lambda);
    if has_connections
        last_psi_history(i) = psi_i_raw;
        fprintf('    导弹 %d: ψ_i = %.4f (更新历史值)\n', i, psi_i_raw);
    end
end

% 然后断开连接
fprintf('  断开连接后：\n');
for i = 1:2
    [psi_i_raw, has_connections] = information_credibility_factor(z_observer, [], zeros(M), i, lambda);
    if ~has_connections
        psi_i = last_psi_history(i);
        fprintf('    导弹 %d: ψ_i = %.4f (保持历史值)\n', i, psi_i);
    end
end

disp('信息可信因子测试完成！');
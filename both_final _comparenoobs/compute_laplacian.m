function L = compute_laplacian(a)
% 计算拉普拉斯矩阵
% 输入：a - 邻接矩阵 (M x M)
% 输出：L - 拉普拉斯矩阵 (M x M)
% 计算规则：
%   l_ij = -a_ij, 当 i ≠ j
%   l_ii = Σ(k=1 to M) a_ik, 当 i = j

M = size(a, 1);
L = zeros(M, M);

for i = 1:M
    for j = 1:M
        if i == j
            % 对角线元素：度
            L(i, j) = sum(a(i, :));
        else
            % 非对角线元素：负的邻接矩阵元素
            L(i, j) = -a(i, j);
        end
    end
end

end



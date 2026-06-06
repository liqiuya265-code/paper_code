%% 圆柱参数
P0 = [1 2 3];    % 圆柱底面中心
P1 = [4 5 7];    % 圆柱顶面中心
r = 1;           % 半径

%% 计算方向向量
v = P1 - P0;
L = norm(v);     % 长度
v = v / L;       % 单位向量

%% 生成沿 z 轴的圆柱网格
N_theta = 50;    % 圆周分辨率
N_z = 50;        % 高度分辨率
theta = linspace(0, 2*pi, N_theta);
z_lin = linspace(0, L, N_z);
[Theta, Z] = meshgrid(theta, z_lin);

X = r * cos(Theta);
Y = r * sin(Theta);

%% 旋转矩阵：将 z 轴旋转到 v 方向
z_axis = [0 0 1];
axis_rot = cross(z_axis, v);
angle = acos(dot(z_axis, v));

if norm(axis_rot) < 1e-6
    R = eye(3); % 已经对齐
else
    axis_rot = axis_rot / norm(axis_rot);
    K = [  0         -axis_rot(3)  axis_rot(2);
          axis_rot(3) 0           -axis_rot(1);
         -axis_rot(2) axis_rot(1) 0         ];
    R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
end

%% 将圆柱点旋转并平移到 P0
pts = R * [X(:)'; Y(:)'; Z(:)'];
Xr = reshape(pts(1,:), size(X)) + P0(1);
Yr = reshape(pts(2,:), size(Y)) + P0(2);
Zr = reshape(pts(3,:), size(Z)) + P0(3);

%% 绘制圆柱表面
figure; hold on;
surf(Xr, Yr, Zr, 'FaceColor','b', 'EdgeColor','none', 'FaceAlpha',1);

%% 绘制封顶和封底
% 顶面
top_center = P1;
fill3(Xr(end,:), Yr(end,:), Zr(end,:), 'b');

% 底面
bottom_center = P0;
fill3(Xr(1,:), Yr(1,:), Zr(1,:), 'b');

%% 美化显示
axis equal;
view(3);
camlight; lighting gouraud;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('实心圆柱体（带封顶封底）');

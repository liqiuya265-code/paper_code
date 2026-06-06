classdef obstacles < handle
    % 障碍物类定义
    % 包含球形和圆柱形障碍物的检测和避免功能

    properties
        % 球形障碍物参数
        spherical_centers = {};  % 球心位置列表
        spherical_radii = [];    % 球形障碍物半径列表
        d_safe = 100;            % 安全距离

        % 圆柱形障碍物参数
        cylindrical_centers = {};  % 圆柱轴线上一点的位置列表
        cylindrical_radii = [];    % 圆柱半径列表
        cylindrical_axes = {};     % 圆柱轴线方向单位矢量列表

        % 控制障碍函数参数
        kappa1 = 1;            % CBF参数 κ₁
        kappa2 = 1;            % CBF参数 κ₂
    end

    methods
        function obj = obstacles(d_safe, kappa1, kappa2)
            % 构造函数
            % d_safe: 安全距离
            % kappa1: CBF参数 κ₁
            % kappa2: CBF参数 κ₂
            if nargin > 0
                obj.d_safe = d_safe;
            end
            if nargin > 1
                obj.kappa1 = kappa1;
            end
            if nargin > 2
                obj.kappa2 = kappa2;
            end
        end

        function add_spherical_obstacle(obj, center, radius)
            % 添加球形障碍物
            % center: [x, y, z] 球心位置
            % radius: 球形半径
            obj.spherical_centers{end+1} = center;
            obj.spherical_radii(end+1) = radius;
        end

        function add_cylindrical_obstacle(obj, center, radius, axis)
            % 添加圆柱形障碍物
            % center: [x, y, z] 圆柱轴线上的一点
            % radius: 圆柱半径
            % axis: [nx, ny, nz] 轴线方向单位矢量
            obj.cylindrical_centers{end+1} = center;
            obj.cylindrical_radii(end+1) = radius;
            obj.cylindrical_axes{end+1} = axis / norm(axis);  % 归一化
        end

        function h_spherical = spherical_barrier_function(obj, p_i, obstacle_idx)
         p_i_col = p_i(:); % 强制转为列向量
         p_o = obj.spherical_centers{obstacle_idx};
            p_o_col = p_o(:); % 强制转为列向量
            r_o = p_i_col - p_o_col;
            r_o_norm_sq = norm(r_o)^2;
            h_spherical = r_o_norm_sq - (obj.d_safe+obj.spherical_radii(obstacle_idx))^2;
         end

function h_cylindrical = cylindrical_barrier_function(obj, p_i, obstacle_idx)
    p_i_col = p_i(:); % 强制转为列向量
    p_c = obj.cylindrical_centers{obstacle_idx};
    p_c_col = p_c(:);
    n_c = obj.cylindrical_axes{obstacle_idx};
    n_c_col = n_c(:);
    r_o = p_i_col - p_c_col;
    P = eye(3) - n_c_col * n_c_col';
    r_bot = P * r_o;
    h_cylindrical = norm(r_bot)^2 - (obj.d_safe+obj.cylindrical_radii(obstacle_idx))^2;
end

        function [min_h_spherical, closest_spherical_idx] = check_spherical_obstacles(obj, p_i)
            % 检查所有球形障碍物，返回最小障碍函数值和最近障碍物索引
            min_h_spherical = inf;
            closest_spherical_idx = 0;

            for i = 1:length(obj.spherical_centers)
                h = obj.spherical_barrier_function(p_i, i);
                if h < min_h_spherical
                    min_h_spherical = h;
                    closest_spherical_idx = i;
                end
            end
        end

        function [min_h_cylindrical, closest_cylindrical_idx] = check_cylindrical_obstacles(obj, p_i)
            % 检查所有圆柱形障碍物，返回最小障碍函数值和最近障碍物索引
            min_h_cylindrical = inf;
            closest_cylindrical_idx = 0;

            for i = 1:length(obj.cylindrical_centers)
                h = obj.cylindrical_barrier_function(p_i, i);
                if h < min_h_cylindrical
                    min_h_cylindrical = h;
                    closest_cylindrical_idx = i;
                end
            end
        end

        function [avoidance_force, obstacle_detected] = compute_obstacle_avoidance(obj, p_i, v_i, a_N)
            % 计算基于CBF的障碍物避免力
            % p_i: 导弹当前位置 [x, y, z]
            % v_i: 导弹速度矢量 [vx, vy, vz]
            % a_N: 名义控制输入（PNG加速度）[ax, ay, az]
            % 返回: avoidance_force - 避免力矢量, obstacle_detected - 是否检测到障碍物
            avoidance_force = [0; 0; 0];
            obstacle_detected = false;
            % 计算导弹速度大小
            V_i = norm(v_i);
            % 检查球形障碍物
            for i = 1:length(obj.spherical_centers)
                h = obj.spherical_barrier_function(p_i, i);
                % fprintf('球形障碍物 %d: h = %.2f\n', i, h);

                p_o = obj.spherical_centers{i};
                r_o_vec = p_i - p_o';
                r_o = norm(r_o_vec);
                if r_o > 1e-6
                    % 从障碍物指向导弹的单位矢量
                    e_r = r_o_vec / r_o;
                    M=eye(3)-v_i*v_i'/(v_i'*v_i);
                    r_eff=M*r_o_vec;
                    % 计算相对速度
                    if norm(r_eff) < 1e-2
                    % 找一个与当前速度严格正交的任意方向作为逃逸引导
                    v_rand = cross(v_i, [1; 0; 0]);
                    if norm(v_rand) < 1e-3
                        v_rand = cross(v_i, [0; 1; 0]);
                    end
                    % 赋予一个微小的偏置，防止分母炸裂
                    r_eff = (v_rand / norm(v_rand)) * 1e-2; 
                end
                    v_rel = v_i;  % 假设障碍物静止
                    r_o_dot = dot(e_r, v_rel);
                    phi1=2*r_o*r_o_dot+obj.kappa1*h;
                    % CBF避免项计算
                    if 2*V_i^2 + 2*(obj.kappa1 + obj.kappa2)*r_o*r_o_dot + obj.kappa1*obj.kappa2*h+2*r_o_vec'*a_N >0
                        a_S_magnitude=0;
                    else
                        numerator = -2*V_i^2 - 2*(obj.kappa1 + obj.kappa2)*r_o*r_o_dot - obj.kappa1*obj.kappa2*h-2*r_o_vec'*a_N;
                        denominator = 2*norm(r_eff)^2;
                        a_S_magnitude = numerator / denominator;
                        obstacle_detected = true;
                    end
                    % 避免力方向沿径向
                    avoidance_force = avoidance_force + a_S_magnitude * r_eff;
                end
            end

            % 检查圆柱形障碍物
            for i = 1:length(obj.cylindrical_centers)
                h = obj.cylindrical_barrier_function(p_i, i);

                p_c = obj.cylindrical_centers{i};
                n_c = obj.cylindrical_axes{i};

                r_o = p_i - p_c';
                P = eye(3) - n_c' * n_c;
                M=eye(3)-v_i*v_i'/(v_i'*v_i);
                r_bot = P * r_o;
                r_bot_norm = norm(r_bot);

                if r_bot_norm > 1e-6
                    % 径向单位矢量
                    e_bot = r_bot / r_bot_norm;
                    r_eff=M*r_bot;
                    % 径向速度分量
                    v_bot_i = P * v_i;
                    v_bot_norm = norm(v_bot_i);
                    % Ψ = 2|v_bot|² + 2·r_bot·a_N + 2(κ₁+κ₂)·r_bot·v_i + κ₁κ₂·h
                    if 2*v_bot_norm^2 + 2*dot(r_bot, a_N) + 2*(obj.kappa1 + obj.kappa2)*dot(r_bot, v_i) + obj.kappa1*obj.kappa2*h > 0
                        a_S_magnitude=0;
                    else
                    % CBF避免项计算  u_s = -Ψ/(2|r_eff|²) · r_eff
                    numerator = -2*v_bot_norm^2 - 2*dot(r_bot, a_N) - 2*(obj.kappa1 + obj.kappa2)*dot(r_bot, v_i) - obj.kappa1*obj.kappa2*h;
                    denominator = 2*norm(r_eff)^2;
                    a_S_magnitude = numerator / denominator;
                    obstacle_detected = true;
                    end
                    % 避免力方向沿径向
                    avoidance_force = avoidance_force + a_S_magnitude * r_eff;
                end

            end

            % % 限制避免力的最大值
            % max_avoidance_force = 100;  % 最大避免力 (m/s^2)
            % if norm(avoidance_force) > max_avoidance_force
            %     avoidance_force = max_avoidance_force * avoidance_force / norm(avoidance_force);
            % end
        end

        function plot_obstacles(obj)
            hold on;
            delete(findall(gca, 'Type', 'light'));

            % 光照
            light('Position', [0.7 0.8 1],   'Style', 'infinite', 'Color', [0.85 0.8 0.75]);
            light('Position', [-0.5 -0.6 0.2], 'Style', 'infinite', 'Color', [0.35 0.35 0.5]);
            lighting gouraud;

            % 配色
            face_c  = [0.85 0.20 0.15];   % 红色
            alpha_f = 0.5;
            edge_c  = [0.55 0.08 0.05];   % 轮廓

            % ---- 球形 ----
            [xs, ys, zs] = sphere(60);
            [xw, yw, zw] = sphere(16);    % 仅勾勒稀疏经纬线
            for i = 1:length(obj.spherical_centers)
                cx = obj.spherical_centers{i}(1);
                cy = obj.spherical_centers{i}(2);
                cz = obj.spherical_centers{i}(3);
                R  = obj.spherical_radii(i);

                surf(xs*R+cx, ys*R+cy, zs*R+cz, ...
                     'FaceColor', face_c, 'FaceAlpha', alpha_f, ...
                     'EdgeColor', 'none', 'FaceLighting', 'gouraud');
                % 稀疏经纬线
                plot3(xw*R+cx, yw*R+cy, zw*R+cz, '-', ...
                      'Color', [edge_c 0.18], 'LineWidth', 0.35);
                plot3(xw'*R+cx, yw'*R+cy, zw'*R+cz, '-', ...
                      'Color', [edge_c 0.18], 'LineWidth', 0.35);
            end

            % ---- 圆柱 ----
            if isempty(obj.cylindrical_centers), return; end
            n_th  = 64;
            th    = linspace(0, 2*pi, n_th);
            h_len = 6000;   % 全长
            for i = 1:length(obj.cylindrical_centers)
                pc = obj.cylindrical_centers{i};
                R  = obj.cylindrical_radii(i);
                n  = obj.cylindrical_axes{i}(:); n = n/norm(n);

                % 旋转 Z→n
                z0 = [0;0;1]; ax = cross(z0,n); sn = norm(ax); cs = dot(z0,n);
                if sn < 1e-10
                    Rot = eye(3);
                else
                    ax = ax/sn;
                    Kx = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
                    Rot = eye(3) + sn*Kx + (1-cs)*(Kx*Kx);
                end

                % 圆柱面（仅两端，surf 插值）
                [Th, Zz] = meshgrid(th, [0, h_len]);
                pts = Rot * [R*cos(Th(:))'; R*sin(Th(:))'; Zz(:)'];
                Xr = reshape(pts(1,:),2,n_th)+pc(1);
                Yr = reshape(pts(2,:),2,n_th)+pc(2);
                Zr = reshape(pts(3,:),2,n_th)+pc(3);
                surf(Xr, Yr, Zr, 'FaceColor', face_c, 'FaceAlpha', alpha_f, ...
                     'EdgeColor', 'none', 'FaceLighting', 'gouraud');

                % 两端封口
                for sg = [0, 1]
                    pr = Rot*[R*cos(th);R*sin(th);sg*h_len*ones(1,n_th)];
                    fill3(pr(1,:)+pc(1), pr(2,:)+pc(2), pr(3,:)+pc(3), ...
                          face_c, 'FaceAlpha', alpha_f, 'EdgeColor', edge_c, ...
                          'LineWidth', 0.8, 'FaceLighting', 'gouraud');
                end
                % 4条母线
                for ag = 0:pi/2:3*pi/2
                    pl = Rot*[R*cos(ag)*[1 1];R*sin(ag)*[1 1];[0 h_len]];
                    plot3(pl(1,:)+pc(1), pl(2,:)+pc(2), pl(3,:)+pc(3), ...
                          '-', 'Color', [edge_c 0.25], 'LineWidth', 0.5);
                end
            end
        end
    end
end

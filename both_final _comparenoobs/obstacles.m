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
            % 计算球形障碍物的障碍函数
            % p_i: 导弹当前位置 [x, y, z]
            % obstacle_idx: 障碍物索引

            p_o = obj.spherical_centers{obstacle_idx};
            r_o = p_i - p_o';
            r_o_norm_sq = norm(r_o)^2;

            % 障碍函数: h(r_o) = r_o^2 - d_safe^2
            h_spherical = r_o_norm_sq - (obj.d_safe+obj.spherical_radii(obstacle_idx))^2;
        end

        function h_cylindrical = cylindrical_barrier_function(obj, p_i, obstacle_idx)
            % 计算圆柱形障碍物的障碍函数
            % p_i: 导弹当前位置 [x, y, z]
            % obstacle_idx: 障碍物索引

            p_c = obj.cylindrical_centers{obstacle_idx};
            n_c = obj.cylindrical_axes{obstacle_idx};

            % 相对位置矢量
            r_o = p_i - p_c';

            % 投影矩阵 P = I - n_c * n_c'
            P = eye(3) - n_c' * n_c;

            % 径向矢量 r_bot = P * r_o
            r_bot = P * r_o;

            % 障碍函数: h(r_o) = r_bot^2 - d_safe^2
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
                    if 2*dot(r_bot,a_N)+2*v_bot_norm^2+2*obj.kappa1*dot(r_bot, v_i)+obj.kappa2*h>0
                        a_S_magnitude=0;
                    else
                    % CBF避免项计算
                    numerator = -2*v_bot_norm^2 - 2*obj.kappa1*dot(r_bot, v_i) - obj.kappa2*h-2*r_bot'*a_N;
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
            % 绘制所有障碍物
            hold on;

            for i = 1:length(obj.spherical_centers)
                center = obj.spherical_centers{i};
                radius = obj.spherical_radii(i);
                safe_radius = sqrt(radius^2 + obj.d_safe^2);

                % 生成单位球
                [x_unit, y_unit, z_unit] = sphere(20);

                % % 绘制安全边界（半透明）
                % x = x_unit * safe_radius + center(1);
                % y = y_unit * safe_radius + center(2);
                % z = z_unit * safe_radius + center(3);
                % surf(x, y, z, 'FaceColor', 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

                % 绘制实际障碍物
                x = x_unit * radius + center(1);
                y = y_unit * radius + center(2);
                z = z_unit * radius + center(3);
                surf(x, y, z, 'FaceColor', 'red', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            end

            % 绘制圆柱形障碍物
            for i = 1:length(obj.cylindrical_centers)
                center = obj.cylindrical_centers{i};
                radius = obj.cylindrical_radii(i);
                axis_dir = obj.cylindrical_axes{i};
            %    safe_radius = sqrt(radius^2 + obj.d_safe^2);
                safe_radius =radius;
                % 定义圆柱长度（对于无限长圆柱，使用固定长度表示）
                cylinder_length =4000;  % 圆柱显示长度

                % 计算圆柱的两个端点
                P0 = center ;  % 底面中心
                P1 = center + (cylinder_length) * axis_dir;  % 顶面中心

                % 计算方向向量
                v = P1 - P0;
                L = norm(v);
                if L > 1e-6
                    v = v / L;  % 单位向量
                end

                % 生成沿z轴的圆柱网格
                N_theta = 30;  % 圆周分辨率
                N_z = 20;      % 高度分辨率
                theta = linspace(0, 2*pi, N_theta);
                z_lin = linspace(0, L, N_z);
                [Theta, Z] = meshgrid(theta, z_lin);

                X = safe_radius * cos(Theta);
                Y = safe_radius * sin(Theta);

                % 旋转矩阵：将z轴旋转到v方向
                z_axis = [0, 0, 1];
                axis_rot = cross(z_axis, v);
                angle = acos(dot(z_axis, v));

                if norm(axis_rot) < 1e-6
                    R = eye(3);  % 已经对齐
                else
                    axis_rot = axis_rot / norm(axis_rot);
                    K = [0, -axis_rot(3), axis_rot(2);
                        axis_rot(3), 0, -axis_rot(1);
                        -axis_rot(2), axis_rot(1), 0];
                    R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
                end

                % 将圆柱点旋转并平移到P0
                pts = R * [X(:)'; Y(:)'; Z(:)'];
                Xr = reshape(pts(1,:), size(X)) + P0(1);
                Yr = reshape(pts(2,:), size(Y)) + P0(2);
                Zr = reshape(pts(3,:), size(Z)) + P0(3);

                % 绘制圆柱表面（半透明）
                surf(Xr, Yr, Zr, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.1);

                % 绘制实际圆柱表面
                X_actual = radius * cos(Theta);
                Y_actual = radius * sin(Theta);
                pts_actual = R * [X_actual(:)'; Y_actual(:)'; Z(:)'];
                Xr_actual = reshape(pts_actual(1,:), size(X_actual)) + P0(1);
                Yr_actual = reshape(pts_actual(2,:), size(Y_actual)) + P0(2);
                Zr_actual = reshape(pts_actual(3,:), size(Z)) + P0(3);

                surf(Xr_actual, Yr_actual, Zr_actual, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

                % 绘制上下封顶
                % 顶面（上端面）
                fill3(Xr(end,:), Yr(end,:), Zr(end,:), 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
                fill3(Xr_actual(end,:), Yr_actual(end,:), Zr_actual(end,:), 'red', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

                % 底面（下端面）
                fill3(Xr(1,:), Yr(1,:), Zr(1,:), 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
                fill3(Xr_actual(1,:), Yr_actual(1,:), Zr_actual(1,:), 'red', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

                % 绘制圆柱体轴线
                plot3([P0(1), P1(1)], [P0(2), P1(2)], [P0(3), P1(3)], 'r-', 'LineWidth', 2);
            end
        end
    end
end

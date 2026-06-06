function [a_now,downtime,attack_state,event_count,last_event_time] = apply_dos(a_last,t_now,dt,a_base,zeta,mu,kappa,nu,downtime,attack_state,event_count,last_event_time,t0,~)
% 根据 |D_{ij}(t0,t)| <= zeta_ij + mu_ij*(t-t0) 和 |F_{ij}(t0,t)| <= kappa_ij + (t-t0)/nu_ij 更新通信邻接矩阵
% 攻击时刻令 a_ij = 0
% 输入：
%   kappa - 频率约束的干扰边界（DoS事件的初始数量）
%   nu - 平均停留时间（连续DoS事件之间的最小时间间隔），需满足 nu > 1
%   event_count - DoS事件发生次数
%   last_event_time - 上次DoS事件发生的时间
[row_n,col_n]=size(a_base);
for r=1:row_n
    for c=1:col_n
        if a_base(r,c)==0
            attack_state(r,c)=0;
            continue;
        end
        % 检查占空比约束：|D_{ij}(t0,t)| <= zeta_ij + mu_ij*(t-t0)
        mu_clamped=min(max(mu(r,c),0),1);         % 占空比上限，限制在 [0,1]
        allowed_duration=zeta(r,c)+mu_clamped*(t_now-t0);  % 允许的最大累计中断
        allowed_duration=max(allowed_duration,0);
        remaining_duration=allowed_duration-downtime(r,c);

        % 检查频率约束：|F_{ij}(t0,t)| <= kappa_ij + (t-t0)/nu_ij
        % 允许的最大DoS事件数
        allowed_events=kappa(r,c)+(t_now-t0)/nu(r,c);
        allowed_events=max(allowed_events,0);
        remaining_events=allowed_events-event_count(r,c); 
        if a_last(r,c)==0 && a_base(r,c)==1
            attack_state_last(r,c)=1;
        else
            attack_state_last(r,c)=0;
        end
        % 最坏情况DoS攻击：尽可能达到上限 |D_{ij}(t0,t)| = zeta_{ij} + mu_{ij}*(t-t0)
        % 需同时满足：
        % 1. 占空比约束：剩余中断时长 > 0
        % 2. 频率约束：剩余事件数 > 0 且距离上次事件时间 >= 最小时间间隔
        can_trigger_duration=(remaining_duration>0);
     %   can_trigger_frequency=(remaining_events>0 && (last_event_time(r,c)==0 || time_since_last_event>=min_interval));
        can_trigger_frequency=(remaining_events>1||(remaining_events>0 && attack_state_last(r,c)==1));
        % 最坏情况：只要满足约束条件，就立即触发攻击（不使用随机概率）
        if can_trigger_duration && can_trigger_frequency
           if attack_state_last(r,c)==0 && rand>0.5
            attack_state(r,c)=1;
            downtime(r,c)=downtime(r,c)+dt;
            if a_last(r,c)==0 && a_base(r,c)==1
                event_count(r,c)=event_count(r,c);  % 增加事件计数
            else
                event_count(r,c)=event_count(r,c)+1; 
            end
            last_event_time(r,c)=t_now;  % 更新上次事件时间
            a_now(r,c)=0; % 攻击使得链路断开
           elseif  attack_state_last(r,c)==1
               attack_state(r,c)=1;
            downtime(r,c)=downtime(r,c)+dt;
            if a_last(r,c)==0 && a_base(r,c)==1
                event_count(r,c)=event_count(r,c);  % 增加事件计数
            else
                event_count(r,c)=event_count(r,c)+1; 
            end
            last_event_time(r,c)=t_now;  % 更新上次事件时间
            a_now(r,c)=0; % 攻击使得链路断开
           else
            attack_state(r,c)=0;
            a_now(r,c)=a_base(r,c);
           end
        else
            attack_state(r,c)=0;
            a_now(r,c)=a_base(r,c);
        end
    end
end


function best_traj = TrajectoryPlanner_createTrajectories(x, y, theta, vx, vy, vtheta, acc_x, acc_y, acc_theta)
%% 参数列表：
% x,y,theta: 机器人当前位姿
% vx, vy, vtheta：机器人当前线速度和角速度
% acc_x, acc_y, acc_theta: x加速度,y加速度,theta加速度
    global g
%% init for product trajectorys
    max_vel_x = g.max_vel_x_;
    if g.final_goal_position_valid_   %如果final_goal_x_ 和 final_goal_y_有有效值
        final_goal_dist = hypot(g.final_goal_x_ - x, g.final_goal_y_ - y );
        max_vel_x = min( max_vel_x, final_goal_dist / g.sim_time_ );
    end
    if (g.dwa_) 
        max_vel_x = max(min(max_vel_x, vx + acc_x * g.sim_period_), g.min_vel_x_);
        min_vel_x = max(g.min_vel_x_, vx - acc_x * g.sim_period_);
        max_vel_theta = min(g.max_vel_th_, vtheta + acc_theta * g.sim_period_);
        min_vel_theta = max(g.min_vel_th_, vtheta - acc_theta * g.sim_period_);
    else 
        max_vel_x = max(min(max_vel_x, vx + acc_x * g.sim_time_), g.min_vel_x_);
        min_vel_x = max(g.min_vel_x_, vx - acc_x * g.sim_time_);
        max_vel_theta = min(g.max_vel_th_, vtheta + acc_theta * g.sim_time_);
        min_vel_theta = max(g.min_vel_th_, vtheta - acc_theta * g.sim_time_);
    end
    %为产生轨迹集初始化角速度和线速度，并且初始化两条轨迹间增加的角速度和线速度
    dvx = (max_vel_x - min_vel_x) / (g.vx_samples_ - 1);
    dvtheta = (max_vel_theta - min_vel_theta) / (g.vtheta_samples_ - 1);
    best_traj = g.traj_one;
    comp_traj = g.traj_one;
    impossible_cost = 100; %path_map_.obstacleCosts();
    
%% generate vx_samples_ * vtheta_samples_ trajectorys, and choose the best
    vx_samp = min_vel_x;
    vtheta_samp = min_vel_theta;
    vy_samp = 0.0;
    
    if ~g.escaping_
        for i =1: g.vx_samples_
            vtheta_samp = 0;
            [comp_traj, ~]= TrajectoryPlanner_generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, ...
                acc_x, acc_y, acc_theta, impossible_cost);
            if(comp_traj.cost_ >= 0 && (comp_traj.cost_ < best_traj.cost_ || best_traj.cost_ < 0))
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
            end
            vtheta_samp = min_vel_theta;
            %next sample all theta trajectories
            for j =1: g.vtheta_samples_ - 1
              [comp_traj, ~]= TrajectoryPlanner_generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, ...
                  acc_x, acc_y, acc_theta, impossible_cost);

              %if the new trajectory is better... let's take it
              if(comp_traj.cost_ >= 0 && (comp_traj.cost_ < best_traj.cost_ || best_traj.cost_ < 0))
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
              end
              vtheta_samp = vtheta + dvtheta;
            end
            vx_samp = vx_samp + dvx;
        end
    end
    
%% next we want to generate trajectories for rotating in place
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;
    heading_dist = inf;
    
    for i =1: g.vtheta_samples_
      if vtheta_samp > 0
         vtheta_samp_limited = max(vtheta_samp, g.min_in_place_vel_th_);
      else
         vtheta_samp_limited = min(vtheta_samp, -1.0 * g.min_in_place_vel_th_);
      end
      [comp_traj, ~]= TrajectoryPlanner_generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited, ...
          acc_x, acc_y, acc_theta, impossible_cost);
      % if the new trajectory is better... let's take it...
      % note if we can legally rotate in place we prefer to do that rather than move with y velocity
      if(comp_traj.cost_ >= 0 ...
          && (comp_traj.cost_ <= best_traj.cost_ || best_traj.cost_ < 0 || best_traj.yv_ ~= 0.0) ...
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta))

        [x_r, y_r, th_r]=getEndpoint(comp_traj);
        x_r = x_r + g.heading_lookahead_ * cos(th_r);
        y_r = y_r + g.heading_lookahead_ * sin(th_r);
        
        % if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) 
            %ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
            ahead_gdist = hypot(g.final_goal_x_ - x_r, g.final_goal_y_ - y_r );
            if (ahead_gdist < heading_dist) %寻找离goal最近的末尾点
                % if we haven't already tried rotating left since we've moved forward
                if (vtheta_samp < 0 && ~g.stuck_left) %如果采样轨迹右转且之前不曾尝试左转 才可赋值到best_traj
                    swap = best_traj;
                    best_traj = comp_traj;
                    comp_traj = swap;
                    heading_dist = ahead_gdist;
                % if we haven't already tried rotating right since we've moved forward
                elseif(vtheta_samp > 0 && ~g.stuck_right) %如果采样轨迹左转且之前不曾尝试右转 才可赋值到best_traj
                    swap = best_traj;
                    best_traj = comp_traj;
                    comp_traj = swap;
                    heading_dist = ahead_gdist;
                end
            end
      end
      vtheta_samp = vtheta_samp + dvtheta;
    end
    
    % do we have a legal trajectory
    if (best_traj.cost_ >= 0) 
        if ( ~ (best_traj.xv_ > 0)) 
            if (best_traj.thetav_ < 0) %右转
                if (g.rotating_right) %正在右转：stuck_right为真
                    g.stuck_right = true;
                end
                g.rotating_right = true;
            elseif (best_traj.thetav_ > 0) %左转
                if (g.rotating_left)
                    g.stuck_left = true;
                end
                g.rotating_left = true;    
            elseif(best_traj.yv_ > 0) 
                if (g.strafe_right) 
                    g.stuck_right_strafe = true;
                end
                g.strafe_right = true;
            elseif(best_traj.yv_ < 0)
                if (g.strafe_left) 
                    g.stuck_left_strafe = true;
                end
                g.strafe_left = true;
            end
            % set the position we must move a certain distance away from
            g.prev_x_ = x;
            g.prev_y_ = y;
        end
        
        dist = hypot(x - g.prev_x_, y - g.prev_y_);
        %oscillation_reset_dist_：机器人必须行进的距离才能探索过去不成功的旋转速度
        %如果机器人当前距离与设置震荡前的距离大于震荡重设距离，则重设所有震荡（重新探索）
        if (dist > g.oscillation_reset_dist_) 
            g.rotating_left = false;
            g.rotating_right = false;
            g.strafe_left = false;
            g.strafe_right = false;
            g.stuck_left = false;
            g.stuck_right = false;
            g.stuck_left_strafe = false;
            g.stuck_right_strafe = false;
        end
        
        dist = hypot(x - g.escape_x_, y - g.escape_y_);
        if(dist > g.escape_reset_dist_ || ...
          fabs(g.escape_theta_ - theta) > escape_reset_theta_)    % shortest_angular_distance(g.escape_theta_ - theta)
            g.escaping_ = false;
        end
        
        return  % find the best_traj from !escaping traj and rotation in place traj
    end
    
%% and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
    vtheta_samp = 0.0;
    vx_samp = g.backup_vel_;	%用于后退的速度
    vy_samp = 0.0;
    [comp_traj, ~]= TrajectoryPlanner_generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, ...
        acc_x, acc_y, acc_theta, impossible_cost);
    
    % we'll allow moving backwards slowly even when the static map shows it as blocked
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
    
    dist = hypot(x - g.prev_x_, y - g.prev_y_);
    if (dist > g.oscillation_reset_dist_) 
        g.rotating_left = false;
        g.rotating_right = false;
        g.strafe_left = false;
        g.strafe_right = false;
        g.stuck_left = false;
        g.stuck_right = false;
        g.stuck_left_strafe = false;
        g.stuck_right_strafe = false;
    end
    
    % only enter escape mode when the planner has given a valid goal point
    if (~g.escaping_ && best_traj.cost_ > -2.0) 
      g.escape_x_ = x;
      g.escape_y_ = y;
      g.escape_theta_ = theta;
      g.escaping_ = true;
    end
    
    dist = hypot(x - g.escape_x_, y - g.escape_y_);

    if (dist > g.escape_reset_dist_ || ...
        abs(g.escape_theta_ - theta) > g.escape_reset_theta_)  % shortest_angular_distance(g.escape_theta_ - theta)
      g.escaping_ = false;
    end
    
    % if the trajectory failed because the footprint hits something, we're still going to back up
    if(best_traj.cost_ == -1.0)
      best_traj.cost_ = 1.0;
    end
end
    
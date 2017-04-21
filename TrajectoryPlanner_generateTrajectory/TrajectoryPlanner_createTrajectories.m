function best_traj = TrajectoryPlanner_createTrajectories(x, y, theta, vx, vy, vtheta, acc_x, acc_y, acc_theta)
%% 参数列表：
% x,y,theta: 机器人当前位姿
% vx, vy, vtheta：机器人当前线速度和角速度
% acc_x, acc_y, acc_theta: x加速度,y加速度,theta加速度
    global g
    max_vel_x = g.max_vel_x_;
    if final_goal_position_valid_   %如果final_goal_x_ 和 final_goal_y_有有效值
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
    vx_samp = min_vel_x;
    vtheta_samp = min_vel_theta;
    vy_samp = 0.0;
    best_traj = g.traj_one;
    comp_traj = g.traj_one;
    impossible_cost = 100; %path_map_.obstacleCosts();
    
    if ~escaping_
        for i =1: vx_samples_
            [comp_traj, ~]= TrajectoryPlanner_generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, ...
            acc_x, acc_y, acc_theta, impossible_cost);
            if(comp_traj.cost_ >= 0 && (comp_traj.cost_ < best_traj.cost_ || best_traj.cost_ < 0))
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
            end
        end
    end
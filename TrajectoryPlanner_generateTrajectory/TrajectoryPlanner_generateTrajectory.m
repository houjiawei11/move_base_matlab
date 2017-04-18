function traj = TrajectoryPlanner_generateTrajectory(x, y, theta,vx, vy, vtheta,vx_samp, vy_samp, vtheta_samp,acc_x, acc_y, acc_theta, impossible_cost)   
%% 参数列表：
% x,y,theta: 机器人当前位姿
% vx, vy, vtheta：机器人当前线速度和角速度
% vx_samp: The x / y / theta velocity used to seed the trajectory
% acc_x, acc_y, acc_theta: x加速度,y加速度,theta加速度
% impossible_cost：最大允许cost
%% 全局变量设置（参数设置）：
    sim_time_ = 0.1;    %采样周期
    sim_granularity_ = 0.05; angular_sim_granularity_ = pi/180.0;   %仿真点间距离
    heading_scoring_=False;
%% 初始化
    x_i = x; y_i = y; theta_i = theta;          %轨迹第一个点的位置
    vx_i = vx; vy_i = vy; vtheta_i = vtheta;    %轨迹第一个点的速度
    vmag = sqrt(vx_samp^2 + vy_samp^2);         %轨迹第一个点的线速度
    if ~heading_scoring_
        num_steps = floor(max((vmag * sim_time_) / sim_granularity_, abs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    else
        num_steps = floor(sim_time_ / sim_granularity_ + 0.5);
    end
    dt = sim_time_ / num_steps;     %两步间时间间隔
    time = 0.0;                     %轨迹第一个点的时间
    %创建一个轨迹（初始化）
    traj.x=[]; traj.y=[]; traj.th=[];
    traj.xv_ = vx_samp; traj.yv_ = vy_samp; traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;
%     %initialize the costs for the trajectory
%     path_dist = 0.0;
%     goal_dist = 0.0;
%     occ_cost = 0.0;
%     heading_diff = 0.0;
%% 产生num_steps个轨迹点
    for i=1:num_steps
%         costmap_.worldToMap(x_i, y_i, cell_x, cell_y):pass
%         if simple_attractor_ 
%            goal_dist = 当前位置跟目标的欧式距离
%         else 
%            %pass
%         end

        %把点加入轨迹
        traj.x=[traj.x x_i];
        traj.y=[traj.y y_i];
        traj.th=[traj.th theta_i];
        %计算新速度
        vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
        vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
        vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);
        %计算新位置
        x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
        y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
        theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);
        time = time + dt;
    end
%% 计算轨迹cost
    %pass
end
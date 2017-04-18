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
    traj=[];
end
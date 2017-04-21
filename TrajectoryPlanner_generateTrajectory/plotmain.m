%% 全局变量设置
global g;
g.sim_time_ = 3.6;    %采样周期
g.sim_period_ = 0.1;
g.sim_granularity_ = 0.4;       g.angular_sim_granularity_ = pi/180.0;   %仿真点间距离
g.heading_scoring_ = false;                                 %评分基于heading方法(false:基于rollout方法)
g.final_goal_x_ = 10.0;         g.final_goal_y_ = 10.0;     g.final_goal_position_valid_ = true;
g.vx_samples_ = 3;              g.vtheta_samples_ = 20;     %要采样的线速度/角速度数量 
g.max_vel_x_ = 1.0;             g.min_vel_x_ = 0.0;
g.max_vel_th_ = 0.1;            g.min_vel_th_ = 0.0;
g.final_goal_position_valid_ = true;                        %是否设定目标点
g.final_goal_x_ = 11.0;         g.final_goal_y_=11.0; 
g.dwa_ = false;                 %用何种方法产生轨迹：dwa/trajectory rollout

% 设置一个轨迹traj_one用于初始化轨迹
g.traj_one.x = []; g.traj_one.y = []; g.traj_one.th = [];
g.traj_one.xv_ = g.min_vel_x_;  g.traj_one.yv_ = 0;  g.traj_one.thetav_ = g.min_vel_th_;
g.traj_one.cost_ = -1.0;

g.min_in_place_vel_th_ = 0.05;
g.heading_lookahead_ = 0.325;   % 往前看多远(0.325m)
g.rotating_right = false;       g.rotating_left = false;    %the traj direction when vy=0
g.stuck_left = false;           g.stuck_right = false;
g.strafe_right = false;         g.strafe_left = false;      %the traj direction when vy>0
g.stuck_right_strafe = false;   g.stuck_left_strafe = false; 
g.prev_x_ = 0.0;                g.prev_y_ = 0.0;            %Used to calculate the distance the robot has traveled before reseting oscillation booleans

g.escaping_ = false;
g.escape_x_=0.0;                g.escape_y_=0.0;            g.escape_theta_=0.0;
g.escape_reset_theta_ = 0.2;    g.escape_reset_dist_ = 0.3;

g.backup_vel_ = -0.4;
g.oscillation_reset_dist_ = 0.3;

%% 创建轨迹

%[tc, num_steps]=TrajectoryPlanner_generateTrajectory(4.5, 4.5, 3.14*0.5, 0.0, 0, 0.04, 0.4, 0, 0.1, 0.4, 0, 0.3, 100);
x=0.0; y=0.7; theta=-0.1;
vx=0.4; vy=0.0; vtheta=0.4;
vx_samp=0.4;vy_samp=0.0; vtheta_samp=0.1;
acc_x=0.5; acc_y=0.0; acc_theta=0.2;
impossible_cost=100;

%[tc, num_steps]=TrajectoryPlanner_generateTrajectory(x,y,theta,vx,vy,vtheta,vx_samp,vy_samp,vtheta_samp, ...
%    acc_x, acc_y, acc_theta, impossible_cost);
%plotTrajectory(tc, num_steps);
best_traj = TrajectoryPlanner_createTrajectories(x, y, theta, vx, vy, vtheta, acc_x, acc_y, acc_theta);

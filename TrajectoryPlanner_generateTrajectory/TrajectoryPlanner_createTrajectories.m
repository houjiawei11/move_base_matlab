function best_traj = TrajectoryPlanner_createTrajectories(x, y, theta, vx, vy, vtheta, acc_x, acc_y, acc_theta)
%% 参数列表：
% x,y,theta: 机器人当前位姿
% vx, vy, vtheta：机器人当前线速度和角速度
% acc_x, acc_y, acc_theta: x加速度,y加速度,theta加速度

max_vel_x = g.max_vel_x_;
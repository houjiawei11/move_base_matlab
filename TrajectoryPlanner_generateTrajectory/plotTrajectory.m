function plotTrajectory(traj, num_steps)
    for i = 1:num_steps
        hold off;
        ArrowLength=0.3;
        area=[-1 14 -1 14];
         quiver(traj.x(i),traj.y(i),ArrowLength*cos(traj.th(i)),ArrowLength*sin(traj.th(i)));hold on;
%         plot(result.x(:,1),result.x(:,2),'-b');hold on;
%         plot(goal(1),goal(2),'*r');hold on;
%         plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
%         plot(traj.x(i),traj.y(i));hold on;
        if ~isempty(traj.x)
            for it=1:length(traj.x)
                plot(traj.x(it),traj.y(it),'.r');hold on;
            end
        end
        axis(area);
        grid on;
        drawnow;
    end
end
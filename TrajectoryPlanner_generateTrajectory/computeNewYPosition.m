function y = computeNewYPosition(yi, vx, vy, theta, dt)
	y = yi + (vx * sin(theta) + vy * sin(0.5*pi + theta)) * dt;
end
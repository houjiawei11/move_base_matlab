function x = computeNewXPosition(xi, vx, vy, theta, dt)
    x = xi + (vx * cos(theta) + vy * cos(0.5*pi + theta)) * dt;
end
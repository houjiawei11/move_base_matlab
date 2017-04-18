function v=computeNewVelocity( vg,  vi,  a_max,  dt)
%vg:采样速度
%vi：输入速度 
%a_max：最大加速度 
%dt：每步时间间隔
    if((vg - vi) >= 0) 
        v = min(vg, vi + a_max * dt);
    else 
        v = max(vg, vi - a_max * dt);
    end
end
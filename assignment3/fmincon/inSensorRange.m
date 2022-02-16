function range = inSensorRange(agent_pos,obst_pos,sensor_range)
dist_agent_obst = norm(agent_pos-obst_pos);
if ((dist_agent_obst^2) < sensor_range)
    range = true;
else
    range = false;
end
end
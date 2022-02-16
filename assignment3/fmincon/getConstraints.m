function [vo,ceq] = getConstraints(agent_velo,agent_pos,sensedObstacles,agent_rad,obst_velo,obst_rad,time_sample)
    %fprintf("I got as obstacle:");
    %sensedObstacles
    sensedobst_len = size(sensedObstacles);
    for i = 1:sensedobst_len(1)
          dist_agent_obs = sensedObstacles(i)-agent_pos;
          rel_v = obst_velo(i,:) - agent_velo;
          bigrad = obst_rad+agent_rad;
          vo(i) = -(norm(dist_agent_obs)^2 - dot(dist_agent_obs,rel_v)^2/norm(rel_v)^2 -bigrad^2); 
    end
%     vo
    ceq = [];
end

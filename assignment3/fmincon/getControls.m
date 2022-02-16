function controls = getControls(agent_pos,agent_goal,agent_rad,sensedObstacles,obst_velo,obst_rad,vmax,time_sample)
    init = [0 0];
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [-vmax -vmax];
    ub = [vmax vmax];
    cost = @(v)cost_optvelo(v,agent_goal,vmax,agent_pos);
    fprintf("\nSize:%d\n",size(sensedObstacles,1));
    %cost = @(v)cost_distance(v,agent_pos,agent_goal,time_sample);
    if size(sensedObstacles,1)>0
        fprintf("I will optimize");
        constraints = @(agent_velo)getConstraints(agent_velo,agent_pos,sensedObstacles(1,:),agent_rad,obst_velo,obst_rad,time_sample);
        controls = fmincon(cost,init,A,b,Aeq,beq,lb,ub,constraints);
    else
        controls = fmincon(cost,init,A,b,Aeq,beq,lb,ub);
    end
    controls
end
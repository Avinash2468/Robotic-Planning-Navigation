function cost2 = cost_optvelo(controls,agent_goal,vmax,agent_pos)
    v_unit = (agent_goal-agent_pos);
    v_unit = v_unit/norm(v_unit);
    v_desired = vmax*v_unit;
    cost2 = norm(v_desired-controls);
end
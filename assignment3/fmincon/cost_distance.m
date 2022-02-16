function cost1 = cost_distance(controls,agent_pos,agent_goal,time_sample)
    cost1 = norm(agent_goal-agent_pos-controls*time_sample);
end
function f = plot_figs(agent_pos,agent_rad,agent_goal,obst_pos,obst_rad)
    axis([-3 10 -3 10]);
    %axis equal;
    sensedobst_len = size(obst_pos);
    for i = 1:sensedobst_len(1)
        rectangle('Position',[agent_pos(1)-agent_rad agent_pos(2)-agent_rad agent_rad*2 agent_rad*2],'Curvature',[1,1],'FaceColor','blue');
        hold on;
        rectangle('Position',[obst_pos(i,1)-obst_rad obst_pos(i,2)-obst_rad obst_rad*2 obst_rad*2],'Curvature',[1,1],'FaceColor','red');
        f = plot(agent_goal(1),agent_goal(2),'rx');
        set(gcf,'Position',[600 600 900 900]);
    end
end
clc;clear;
agent_pos = [0 0];
vmax = 0.4;
agent_goal = [8 8];
obst_pos = zeros(2,2);
obst_pos(1,:) = [3.5 3.5];
obst_pos(2,:) = [0 5];
obst_pos(3,:) = [0 3];
obst_pos(4,:) = [0 2.5];
% obst_pos(5,:) = [4.2 8];
% obst_pos(6,:) = [0 7];
% obst_pos(7,:) = [6 10];
% obst_velo = [0.2 0.2 0.5 -0.3 -0.2];
obst_velo(1,:) = [-0.5 -0.5];
obst_velo(2,:) = [0.2 0];
obst_velo(3,:) = [0.2 0];
obst_velo(4,:) = [0.4 0];
% obst_velo(5,:) = [0 -0.4];
% obst_velo(6,:) = [0.2 0];
% obst_velo(7,:) = [0 -0.2];
obst_rad = 0.2;
agent_rad = 0.1;
sensor_range = 5;
time_sample = 0.1;
iter = 1;
agent_velo = [0 0];
agent_pos_list = [];
nobst = 4;
while (norm(agent_pos-agent_goal)>0.1)
    sensedObstacles = [];
    for i = 1:nobst
        collision_flag = 0;
        obst_pos(i,:) = obst_pos(i,:) + time_sample*obst_velo(i,:);
        if (inSensorRange(agent_pos,obst_pos(i,:),sensor_range) && dot((obst_pos(i,:)-agent_pos),(agent_velo))>=0.3)
%             collision_flag = getConstraints(agent_velo,agent_pos,obst_pos(i,:),agent_rad,obst_velo(i,:),obst_rad*1.5,time_sample);
            %fprintf("%dth obstacle: %d\n",i,collision_flag);
%             if (collision_flag>0)
            sensedObstacles= [sensedObstacles;obst_pos(i,:)];
            fprintf("I see obstacles to avoid %d\n:", i);
%             %sensedObstacles
%             end
        end
    end
    %sensedObstacles
    agent_velo = getControls(agent_pos,agent_goal,agent_rad,sensedObstacles,obst_velo,obst_rad+0.7,vmax,time_sample); 
    agent_pos = agent_pos+agent_velo*time_sample;
    agent_pos_list = [agent_pos_list;agent_pos];
    %plot(agent_pos(1),agent_pos(2),'b.');
    %hold on;
    F(iter) = plot_figs(agent_pos,agent_rad,agent_goal,obst_pos,obst_rad);
    basefilename = sprintf('snap%d.png',iter);
    fullname = fullfile('data/',basefilename);
    saveas(F(iter),fullname);
    clf;
    hold on;
    plot(agent_pos_list(:,1),agent_pos_list(:,2),'k*');
    iter=iter+1;
end
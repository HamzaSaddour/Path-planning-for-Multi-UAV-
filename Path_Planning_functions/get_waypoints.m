%creat WP
%this function assemble all the way points in one matrix 
%it aligns the different trajectories by repeating the last point until
%the end of the simulation time 

function waypoints = get_waypoints(UAVs)
nb_UAVs=size(UAVs,2);
UAV_nb_wp=[];
for i =1:nb_UAVs
    UAV_nb_wp=[UAV_nb_wp;size(UAVs{i},1)];
end
max_wp=max(UAV_nb_wp);
waypoints=[];
for i =1:nb_UAVs
    path=UAVs{i};
    path=[path;repmat( path(end,:), max_wp- size(path,1),1)];
    waypoints=[waypoints,path];
end

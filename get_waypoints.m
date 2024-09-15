%creat WP
%this function assemble all the way points in one matrix 
%it allignes the different trajectorires by repeting the last point untill
%the end of the simulation time 

function waypoints = get_waypoints(UAVs)
%uav=convertCharsToStrings(fieldnames(UAVs));
nb_UAVs=size(UAVs,2);
%nb_UAVs =size(uav,1); %number of the UAVs in the structure
UAV_nb_wp=[];
for i =1:nb_UAVs
    %UAV_nb_wp=[UAV_nb_wp;length(UAVs.(uav(i)))];
    UAV_nb_wp=[UAV_nb_wp;size(UAVs{i},1)];
end
max_wp=max(UAV_nb_wp);
waypoints=[];
for i =1:nb_UAVs
    %path=UAVs.(uav(i));
    path=UAVs{i};
    path=[path;repmat( path(end,:), max_wp- size(path,1),1)];
    waypoints=[waypoints,path];
end

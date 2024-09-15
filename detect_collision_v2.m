function [collision,obstcles_pos,list_0_1]= detect_collision_v2 (paths,WP,safety_distance,step_distance)
%waypoints is a cell array with every cell include the 
uav1=paths{1};%the uav in question 
list_0_1=zeros(size(uav1,1),1);
nb_uavs=size(paths,2);
time_steps= size(uav1,1);
obstcles_pos=[];
collision=0;
waypoints=WP;
% paths
l=[];
for i= 1:time_steps %for every time step 
    
    % for j=2:nb_uavs %for every uav position , I've just added the 2 which supposed to be here
    j=2;
    while (j<=nb_uavs)    
        uav2=paths{1,j};
        if (i>size(uav2,1))
            paths(j)=[]; %remove the path as it's no longer needed 
            %attention to the parathesis type 
            waypoints(:,3*j-2:3*j)=[];
            nb_uavs=nb_uavs-1;
            continue
        end

        if norm(uav1(i,1:3)-uav2(i,1:3))<safety_distance %comparing the distance from the first uav to all the other UAVs
            % hold on 
            % l(1)=scatter3(uav1(i,1),uav1(i,2),uav1(i,3),'red','filled','o','DisplayName', 'Danger points',SizeData=50);
            list_0_1(i)=1;
            collision=1;
            % obstcles_pos=[obstcles_pos;uav2(i,1:3)];

        

            pos = reshape(waypoints(i,4:end),3,[])'; %getting the positions of all the uav at that time step with a collision //nb_uavs-1
            distances=vecnorm(pos-waypoints(i,1:3),2,2); % get the distances from the point of collision to all uavs 
            pos=pos(distances<2*safety_distance,:); %take into consideration only the uavs within the sphere of a radius 2*safety distance
            obstcles_pos=[obstcles_pos;pos]; 
            % l(2)=scatter3(pos(:,1),pos(:,2),pos(:,3),'black','filled','o','DisplayName', 'Psedo obstacles positions',SizeData=100);
            break

            %getting the other UAV position at the time t 
            %this can be edited by adding the position of all the other
            %UAVs as obstacles to avoid getting closer to another one when
            %changing the path 
        end
        j=j+1;
    end

end
% legend([l(:)])
nb_point=0;%(safety_distance/2)/step_distance
l=length(list_0_1);
i=1;
while i<l
    if (list_0_1(i)==0 && list_0_1(i+1)==1)
        list_0_1(max(i-nb_point,2):i)=1;%=[start_goal_list;[UAVx_waypoints(max(i-10,1),1:3), max(i-10,1)]];
    elseif (list_0_1(i)==1 && list_0_1(i+1)==0)
        list_0_1(i:min(i+nb_point,l-1))=1;%start_goal_list=[start_goal_list;[UAVx_waypoints(min(i+10,l),1:3), min(i+10,l)]];
        i=i+10;
    end
i=i+1;
end
list_0_1(1)=0;
list_0_1(end)=0;
end

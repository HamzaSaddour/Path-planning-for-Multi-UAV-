%this function return a list of all the possible collision positions and a
%list (list_zero_one) that indicates the segment of the trajectory to be
%rebuild 
function [collision,obstcles_pos,list_0_1]= detect_collision (waypoints,safety_distance)
%waypoints includes the waypoints of the all UAVs for time t , it has the
%form of [x y z x y z x y z x ....]

list_0_1=zeros(size(waypoints,1),1);
nb_uavs=size(waypoints,2)/3;
obstcles_pos=[];
collision=0;
for i= 1:size(waypoints,1) %for every time step 
    for j=4:3:size(waypoints,2)-2 %for every uav position , I've just added the 2 which supposed to be here
        if norm(waypoints(i,1:3)-waypoints(i,j:j+2))<safety_distance %comparing the distance from the first uav to all the other UAVs
            list_0_1(i)=1;
            collision=1;
            % obstcles_pos=[obstcles_pos;waypoints(i,j:j+2)];

            pos = reshape(waypoints(i,4:end),3,nb_uavs-1)'; %getting the positions of all the uav at that time step with a collision 
            distances=vecnorm(pos-waypoints(i,1:3),2,2); % get the distances from the point of collision to all uavs 
            pos=pos(distances<2*safety_distance,:); %take into consideration only the uavs within the sphere of a radius 2*safety distance
            obstcles_pos=[obstcles_pos;pos]; 
            break

            %getting the other UAV position at the time t 
            %this can be edited by adding the position of all the other
            %UAVs as obstacles to avoid getting closer to another one when
            %changing the path 
        end
    end

end
list_0_1(1)=0;
list_0_1(end)=0;
end

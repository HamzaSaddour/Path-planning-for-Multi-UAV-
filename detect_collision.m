%this function return a list of all the possible collision positions and a
%list (list_zero_one) that indicates the segment of the trajectory to be
%rebuild 
function [obstcles_pos,list_0_1]= detect_collision (waypoints,safety_distance)
%waypoints includes the waypoints of the all UAVs for time t
list_0_1=zeros(size(waypoints,1),1);
obstcles_pos=[];
for i= 1:size(waypoints,1)
    for j=4:size(waypoints,2)-2 %nb of UAVs
        if norm(waypoints(i,1:3)-waypoints(i,j:j+2))<safety_distance %comparing the distance with all the other UAVs
            list_0_1(i)=1;
            obstcles_pos=[obstcles_pos;waypoints(i,j:j+2)];%getting the other UAV position at the time t 
            %this can be edited by adding the position of all the other
            %UAVs as obstacles to avoid getting closer to another one when
            %changing the path 
        end
    end

end
end

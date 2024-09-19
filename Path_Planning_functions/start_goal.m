function start_goal_list = start_goal(list1_0,UAVx_waypoints)
%UAVx_waypoints is the way points of the UAV that we're editing the
%trajectory

%list1_0 is a list that has the same size as the waypoints, it's 1 when the
%position is at less that 10m from the obstacle
start_goal_list=[];
for i= 1:length(list1_0)-1
    if list1_0(i) ~= list1_0(i+1)
        start_goal_list=[start_goal_list;[UAVx_waypoints(i,1:3), i]];
    end
end
% l=length(list1_0);
% i=1;
% while i<l
%     if (list1_0(i)==0 && list1_0(i+1)==1)
%         start_goal_list=[start_goal_list;[UAVx_waypoints(max(i-10,1),1:3), max(i-10,1)]];
%     elseif (list1_0(i)==1 && list1_0(i+1)==0)
%         start_goal_list=[start_goal_list;[UAVx_waypoints(min(i+10,l),1:3), min(i+10,l)]];
%     else
%         i=i+1;
%     end
% 
% end
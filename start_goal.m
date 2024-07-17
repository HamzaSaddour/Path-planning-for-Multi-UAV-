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
%One UAV path planning 
%this function returns the waypoints of one UAV
function path = One_path_planning(start,goal,planner,sv,ss,UAV_speed,time_step)

[pthObj,solnInfo] = plan(planner,start,goal);

if (~solnInfo.IsPathFound)
    disp("No Path Found by the RRT")
    %return
end
interpolatedSmoothWaypoints=exampleHelperUAVPathSmoothing(ss,sv,pthObj);
distance= trajectory_distance(interpolatedSmoothWaypoints);
desired_distance= time_step*UAV_speed;
nb_of_point= floor(distance /desired_distance);
interpolate(interpolatedSmoothWaypoints,nb_of_point)
path=interpolatedSmoothWaypoints.States;


end
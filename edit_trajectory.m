function new_traj = edit_trajectory(trajet,ss,map3D,UAV_speed,time_step)
%trajet is a matrix with the trajectory for all UAVs
%UAV is the number of the UAV that we're editing the trajectory/set 1
%this function returns the new waypoints of the first UAV

%If I want to keep the map3D as it is without adding the pseudo obstacles I
%have to load it again 
load("map3D.mat","map3D");
map3D.FreeThreshold = 0.4;
map3D.OccupiedThreshold = 0.7; 
inflate(map3D,1)
ss=MyCustomStateSpace;
waypoints = trajet;
safety_distance=20;
rad=20;%safety_distance;

[obstcles_pos,list_0_1]= detect_collision (waypoints,safety_distance); %obstcles_pos is positions of the other UAV that causes a danger
UAV1_waypoints = trajet(:,1:3); %getting the coordinates of the first UAV

start_goal_list= start_goal(list_0_1,UAV1_waypoints); %this is the staet and end points for parts of the path to edit 
obstcles_pos
%adding the obstacles to the map  %this is the step that modifies the
%original map If I want to use it again 
for i= 1:length(obstcles_pos)
    position=obstcles_pos(i,:);
    [xObstacle,yObstacle,zObstacle] = meshgrid(position(1)-rad:position(1)+rad,position(2)-rad:position(2)+rad,position(3)-rad:position(3)+rad);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    setOccupancy(map3D,xyzObstacles,1)
    %addMesh(scene,"Cylinder", {[position(1) position(2) rad], [position(3)-rad position(3)+rad]}, [1 0 0]); 
end

sv = validatorOccupancyMap3D(ss);
sv.ValidationDistance = 0.1;
sv.Map = map3D;
planner = plannerRRTStar(ss,sv);
planner.MaxConnectionDistance =2;
planner.MaxIterations = 10000;
planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3))<2);
planner.GoalBias = 0.8; 
new_traj_UAV=waypoints(:,1:3);

for i = 1:2:size(start_goal_list,1)-1
    start_idx=start_goal_list(i,4);
    end_idx = start_goal_list(i+1,4);
    start=start_goal_list(i,1:3);
    while checkOccupancy(map3D,start)
        start_idx=start_idx-1;
        start=UAV1_waypoints(start_idx,1:3);
    end
    start
    start_validaty=checkOccupancy(map3D,start)

    goal=start_goal_list(i+1,1:3);
    while checkOccupancy(map3D,goal)
        end_idx=end_idx+1;
        goal=UAV1_waypoints(end_idx,1:3);
    end
    goal
    goal_validaty=checkOccupancy(map3D,goal)
    seg= One_path_planning(start,goal,planner,sv,ss,UAV_speed,time_step);
    new_traj_UAV= [waypoints(1:start_idx-1,1:3);
                        seg(:,1:3);
                      waypoints(end_idx+1:end,1:3)];

end

new_traj = new_traj_UAV;

end



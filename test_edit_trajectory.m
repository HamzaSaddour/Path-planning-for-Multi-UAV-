%test of function edit trajector 
time_step=1;
UAV1_speed=2;
UAV2_speed=2;

load("map3D.mat","map3D");
scene = uavScenario("ReferenceLocation",[(51.4970+51.5003)/2 (-0.1805-0.1705)/2 0]);
xlimits = [-300 300];
ylimits = [-300 300];
altitude = 0;
color = [0.6 0.6 0.6];
addMesh(scene,"terrain",{"gmted2010",xlimits,ylimits},color,Verbose=true);
color = [0 1 0];
addMesh(scene,"buildings",{"ICL_map.osm",xlimits,ylimits,"auto"},color,Verbose=true);
map3D.FreeThreshold = 0.4;
map3D.OccupiedThreshold = 0.7; 
inflate(map3D,1)

ss=MyCustomStateSpace;
sv = validatorOccupancyMap3D(ss);
sv.Map = map3D;
sv.ValidationDistance = 0.5;
rng(2000)
planner = plannerRRTStar(ss,sv);
planner.MaxConnectionDistance =50;
planner.MaxIterations = 50000;
planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3))<10);
planner.GoalBias = 0.8; 
figure;
show3D(scene);
hold on
%%
%path planning for UAV1
start = [-210 -300 70];
goal =[171 126 71];
path1=One_path_planning(start,goal,planner,sv,ss,UAV1_speed,time_step);

scatter3(start(1,1),start(1,2),start(1,3),'g','filled','x') % draw start state
scatter3(goal(1,1),goal(1,2),goal(1,3),'r','filled','o')
plot3(  path1(:,1), ...
        path1(:,2), ...
        path1(:,3), ...
        "LineWidth",2,"Color","b");

%path planning for UAV2
start = [171 180 71];
goal = [-210 -300 70];
path2=One_path_planning(start,goal,planner,sv,ss,UAV2_speed,time_step);
scatter3(start(1,1),start(1,2),start(1,3),'g','filled','x') % draw start state
scatter3(goal(1,1),goal(1,2),goal(1,3),'r','filled','o')
plot3(  path2(:,1), ...
        path2(:,2), ...
        path2(:,3), ...
        "LineWidth",2,"Color","c");
wp.UAV1=path1;
wp.UAV2=path2;
waypoints = get_waypoints(wp);

%% get the new trajectory for UAV1
new_traj = edit_trajectory(waypoints,ss,map3D,UAV1_speed,time_step);%
new_path1=new_traj(:,1:3);

plot3(  new_path1(:,1), ...
        new_path1(:,2), ...
        new_path1(:,3), ...
        "LineWidth",2,"Color","y","MarkerSize",1);
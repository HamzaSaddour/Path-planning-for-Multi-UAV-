%One UAV path planning 
%this function used the proposed optimised single UAV path planning
%algorithm to return the path for given start-end points 

function [final_path,ss,sv,solnInfo,problem] = One_path_planning(start_goal,map3D,UAV_speed,time_step,map_limits,MaxConnectionDistance,min_goal_distance,nb_itirations,mode,fixed_wings,editing)
%the "mode" pararmeter defines the reach goal mode: 1 for 2D diastance from
%the goal 2 for 3D distance from the goal 

%editing indicates if the function is used when editing the paths 
%fixed_wings is 1 for fixed wings UAVs

max_dis=MaxConnectionDistance;
reach=min_goal_distance;
seed=1;
path=[];
final_path=[];
ss=[];
sv=[];
solnInfo=[];
start=start_goal(1,:);
goal=start_goal(2,:);

%% check if the goal and start are valid

if (checkOccupancy(map3D,start)==1)
    disp("the start position is not valid for this UAV")
    return
elseif (checkOccupancy(map3D,goal)==1)
    disp("the goal position is not valid for this UAV")
    return
end

xlimits=map_limits(1,:);
ylimits=map_limits(2,:);
zlimits=map_limits(3,:);
centre= (goal+start)./2;
d=norm(goal-start);

method = 2; % select 1 ot 2 to change the restricting method or 0 for full map search area
switch method
       case 0
          bounds= map_limits;
        case 1
          bounds = [max(centre(1)-d,xlimits(1)) min(centre(1)+d,xlimits(2));
          max(centre(2)-d,ylimits(1)) min(centre(2)+d,ylimits(2));
                        zlimits];
        case 2
            marge=50; 
            bounds =[max(min(start(1),goal(1))-marge,xlimits(1)) min(max(start(1),goal(1))+marge,xlimits(2));
            max(min(start(2),goal(2))-marge,ylimits(1)) min(max(start(2),goal(2))+marge,ylimits(2));
                        zlimits];
end
if editing 
    bounds = [max(centre(1)-d,xlimits(1)) min(centre(1)+d,xlimits(2));
          max(centre(2)-d,ylimits(1)) min(centre(2)+d,ylimits(2));
                        zlimits];
end

% plotting the search area %uncomment the following lines to plot the
% sampling area 

% Xmin=bounds(1,1);Xmax=bounds(1,2);
% Ymin=bounds(2,1);Ymax=bounds(2,2);
% hold on;
% x = [Xmin Xmax Xmax Xmin Xmin];
% y = [Ymin Ymin Ymax Ymax Ymin];
% z=300*ones(1,length(x));
% rec=plot3(x, y, z,'r--', 'LineWidth', 3); 
% fill(x,y, 'white', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% legend('search area');


if fixed_wings
    ss =ExampleHelperUAVStateSpace("MaxRollAngle",pi/3,...
                                "AirSpeed",6,...
                                "FlightPathAngleLimit",[-0.1 0.1],...
                                "Bounds",[bounds;-pi pi]);
    start=[start,pi/2];
    goal=[goal,pi/2];
else
    ss = MyCustomStateSpace(bounds);
end

sv = validatorOccupancyMap3D(ss);
sv.ValidationDistance = 0.1;
sv.Map = map3D;
got_path=0;
while (~got_path)
    planner = plannerRRTStar(ss,sv);
    planner.MaxConnectionDistance =max_dis;
    if (mode == 1) 
        GoalReachedFcn= @(~,x,y)(norm(x(1:2)-y(1:2))<reach);%2D reach goal
        planner.MaxIterations = 5000;
    else
        GoalReachedFcn= @(~,x,y)(norm(x(1:3)-y(1:3))<reach);
        planner.MaxIterations = 10000;

    end

    planner.GoalReachedFcn = GoalReachedFcn;
    planner.GoalBias = 0.2; 
    
    [pthObj,solnInfo,smoothed_path,distance,problem] = planRRTSS(planner,start,goal,nb_itirations,seed,ss,sv);
   
  
    if (~solnInfo.IsPathFound)
        disp("No Path Found, trying again . . .")
        seed=seed+1;
        max_dis=max_dis+5;
        reach=reach+2;
    else
        got_path=1;
    end
end
disp("before smoothing :"+int2str(size(pthObj.States,1)))

smoothpath=smoothed_path;

disp("after smoothing 1 :"+int2str(size(smoothpath,1)))

tension=3;
if editing 
    tension=4;
end
if problem==0 && (size(smoothpath,1)>2)
    points=mat2cell(smoothpath, ones(size(smoothpath,1),1), size(smoothpath,2));
    smoothpath=hobbysplines(points,'debug',true,'cycle',false,'tension',tension);
    if (~check_path(smoothpath,sv))
      smoothpath=smoothed_path;
    end

end

distance= trajectory_distance(smoothpath);
desired_distance= time_step*UAV_speed;
nb_of_point= max(floor(distance /desired_distance),size(smoothpath,1));

smoothWaypointsObj=navPath(ss);
    %add smooth waypoints to the object.
append(smoothWaypointsObj, smoothpath(:,:));

interpolate(smoothWaypointsObj,floor(nb_of_point))

final_path=smoothWaypointsObj.States(1:nb_of_point,:);

%% plots

% hold on 

% p1=plot3(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),solnInfo.TreeData(:,3),'Color','c','LineWidth',1.5,'DisplayName', 'Tree Expansion');
% 
% path =pthObj.States;
%         hold on
%         p2=plot3(  path(:,1), ...
%                 path(:,2), ...
%                 path(:,3), ...
%                 "LineWidth",1.2,"Color",'k','DisplayName', 'Initial Path');
% 
% path =smoothed_path;
% p3=plot3(  path(:,1), ...
%         path(:,2), ...
%         path(:,3), '-*',"LineWidth",2,"Color",'y','DisplayName', 'Pruned Path');
% 
% hold on 
% 
% 
% path =final_path;
% p4=plot3(  path(:,1), ...
%         path(:,2), ...
%         path(:,3),"LineWidth",2,"Color",'b','DisplayName', 'Smoothed Path');
% 
% hold on 


% scatter3(path(:,1),path(:,2),path(:,3),'k','filled','diamond',SizeData=25,MarkerEdgeColor='k')

 %NumNodes :	Number of nodes in the search tree when the planner terminates (excluding the root node). %NumIterations :	Number of "extend" routines executed.

% NumNodes=solnInfo.NumNodes
% NumIterations=solnInfo.NumIterations
% d=solnInfo.PathCosts(end)

% % plotting the search range
% Xmin=bounds(1,1);Xmax=bounds(1,2);
% Ymin=bounds(2,1);Ymax=bounds(2,2);
% 
%     x = [Xmin Xmax Xmax Xmin Xmin];
%     y = [Ymin Ymin Ymax Ymax Ymin];
%     z=300*ones(1,length(x));
%     hold on;
%     plot3(x, y, z,'r--', 'LineWidth', 2,'DisplayName', 'Search Area'); 
%     fill(x,y, 'black', 'FaceAlpha', 0.2, 'EdgeColor', 'none');

% p5=scatter3(start(1),start(2),start(3),'black','filled','o','DisplayName', 'Start');
% p6=scatter3(goal(1),goal(2),goal(3),'white','filled','o','MarkerEdgeColor','k','DisplayName', 'Goal');
% 
% legend([p5,p6]);%p1,p2,p3,p4p1,p2,p4,
end
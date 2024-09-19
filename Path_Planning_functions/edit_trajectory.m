% This function returns the edited path of UAV number *UAV* that maintains a min
%of a safety distance from all the other UAVs along its trajectory 

function [new_traj_UAV,ss,sv] = edit_trajectory(UAV,map3D,UAV_speed,time_step,scene,paths_,safety_distance)%,UAV
ss=[];
sv=[];
edited_cells=[];
cells=[];
%paths_ is an array cell with the trajectory for all UAVs //
%UAV is the number of the UAV that we're editing the trajectory/set 1

nb_itirations=5;

colors =[1 0 0;
         1 0 1;
         0 0 1;
         1 1 0;
         0 1 1;
         0 1 0;
         1 1 1;
         0 0 0];
%%
%rearrange the waypoints to put the positions of the UAV that we want to
%modify first 
waypoints = get_waypoints(paths_);
UAV_i = waypoints(:,1+3*(UAV-1):3+3*(UAV-1));
waypoints(:,1+3*(UAV-1):3+3*(UAV-1))=waypoints(:,1:3);
waypoints(:,1:3)=UAV_i;

paths=paths_(:,UAV:end); %only checking with uav x to n 
%%

rad=safety_distance+2;

%obstcles_pos contains the positions of the other UAV that causes a danger
step_distance= time_step*UAV_speed;%distance travelled every time step
[collision,obstcles_pos,list_0_1]= detect_collision_v2 (paths,waypoints,safety_distance,step_distance); %obstcles_pos is positions of the other UAV that causes a danger


if (~collision) 
    disp("no collision")
    new_traj_UAV=paths_{UAV};
    return
end

UAV1_waypoints = waypoints(:,1:3); %getting the coordinates of the UAV we're changing the path 

start_goal_list= start_goal(list_0_1,UAV1_waypoints); %this is the start and end points for parts of the path to edit 
disp("adding pseudo-obstacles to the occupancy map")
%adding the obstacles to the occupancy map 
for i= 1:size(obstcles_pos,1)
    position=obstcles_pos(i,:);
    [xObstacle,yObstacle,zObstacle] = meshgrid(position(1)-rad:position(1)+rad,position(2)-rad:position(2)+rad,position(3)-rad:position(3)+rad);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    A1 = checkOccupancy(map3D,xyzObstacles);
    cells=xyzObstacles(A1~=1,:); %cells to set as occupied
    if (size(cells,1)~=0)
        setOccupancy(map3D,cells,1)
        edited_cells=[edited_cells;cells]; %these are the cells of all the psudo obstacles to set as free at the end of the function
       
    end
%adding the obstacles to the scenario
half_size = safety_distance;

% Define the vertices of the cube relative to the center position
vertices = [...
    -half_size, -half_size, -half_size;  % Vertex 1
    half_size, -half_size, -half_size;   % Vertex 2
    half_size, half_size, -half_size;    % Vertex 3
    -half_size, half_size, -half_size;   % Vertex 4
    -half_size, -half_size, half_size;   % Vertex 5
    half_size, -half_size, half_size;    % Vertex 6
    half_size, half_size, half_size;     % Vertex 7
    -half_size, half_size, half_size;    % Vertex 8
];

% Offset vertices by the position (center of the cube)
vertices = vertices + position;

% Define the faces of the cube using triangular elements (each quad is split into 2 triangles)
faces = [...
    1, 2, 3; 1, 3, 4;  % Bottom face
    5, 6, 7; 5, 7, 8;  % Top face
    1, 2, 6; 1, 6, 5;  % Front face
    2, 3, 7; 2, 7, 6;  % Right face
    3, 4, 8; 3, 8, 7;  % Back face
    4, 1, 5; 4, 5, 8;  % Left face
];

addMesh(scene, 'Custom', {vertices, faces}, colors(mod(UAV,8)+1,:));%'Position', position,'Color','k' , 'Opacity', 0.5
%%
end
disp("pseudo obstacles added")
seg={};
start_seg_idx=1;
for i = 1:2:size(start_goal_list,1)-1
    disp("index i = "+int2str(i))
    start_idx=start_goal_list(i,4);
    end_idx = start_goal_list(i+1,4);
    start=start_goal_list(i,1:3);
    while (checkOccupancy(map3D,start) && start_idx>1)
        start_idx=start_idx-1;
        start=UAV1_waypoints(start_idx,1:3);
    end
   
    if (i>2)
        
        if (start==start_goal_list(i-2,1:3)) %%this messy part of code is to avoid cases where there are same start end
            end_idx = start_goal_list(i-1,4);
            continue
        end
    end
    start_goal_list(i,1:3)=start;
    start_goal_list(i,4)=start_idx;
    

    goal=start_goal_list(i+1,1:3);
    while (checkOccupancy(map3D,goal) && end_idx<size(UAV1_waypoints,1))
        end_idx=end_idx+1;
        goal=UAV1_waypoints(end_idx,1:3);
    end
    start_goal_list(i+1,1:3)=goal;
    start_goal_list(i+1,4)=end_idx;

    
    setOccupancy(map3D,[start;goal],0);
    map_limits= [-300 300; -300 300; 50 250];
    seg{i}= UAV1_waypoints(start_seg_idx:start_idx-1,1:3);
    hold on
    % scatter3(start(1),start(2),start(3),'green','*','SizeData',50)
    % scatter3(goal(1),goal(2),goal(3),'green','*','SizeData',50)

    %One_path_planning(start_goal,map3D,UAV_speed,time_step,map_limits,MaxConnectionDistance,min_goal_distance,nb_itirations,mode,fixed_wings,editing)
    min_goal_distance=0.5;
    MaxConnectionDistance=5;
    [seg{i+1},ss,sv,solnInfo,problem]= One_path_planning([start;goal],map3D,UAV_speed,time_step,map_limits,MaxConnectionDistance,min_goal_distance,nb_itirations,2,0,1);%One_path_planning(start_goal,map3D,UAV_speed,time_step,map_limits,MaxConnectionDistance,min_goal_distance,nb_itirations,mode,fixed_wings)

    if (~solnInfo.IsPathFound)
        disp("No Path Found for this avoidance")
        seg{i+1}=UAV1_waypoints(start_idx:end_idx,1:3);
        
    end
   
    start_seg_idx=end_idx;
    
end
new_traj_UAV=[];
if size(seg,2)==0
    new_traj_UAV=UAV1_waypoints(:,1:3);
else
    for i =1:size(seg,2)
        new_traj_UAV=[new_traj_UAV;seg{i}];
    end
    
    new_traj_UAV=[new_traj_UAV;UAV1_waypoints(end_idx:end,1:3)];
    
    ss = MyCustomStateSpace([-300 300; -300 300; 50 200]);
    sv = validatorOccupancyMap3D(ss);
    sv.ValidationDistance = 0.1;
    sv.Map = map3D;

    pathObj=navPath(ss);
    append(pathObj, new_traj_UAV(:,1:3));


    % disp("distance...")
    distance= pathLength(pathObj);
    desired_distance= time_step*UAV_speed;
    nb_of_point= max(size(new_traj_UAV,1),floor(distance /desired_distance));

    % disp("interpolate 1 ...")
    interpolate(pathObj,nb_of_point)
    new_traj_UAV=pathObj.States;

    
    [smoothpath,problem] = PathSmoothing_star(sv,new_traj_UAV);%it was by error smoothing not star
    new_traj_UAV=smoothpath;
    
    if problem==0
        disp("hobbysplines smoothing ...")
        points=mat2cell(smoothpath, ones(size(smoothpath,1),1), size(smoothpath,2));
        smoothpath=hobbysplines(points,'debug',true,'cycle',false,'tension',2); 
        if (~check_path(smoothpath,sv))
        smoothpath=new_traj_UAV;
         disp("smoothing removed")
        end
    end

    smoothWaypointsObj=navPath(ss);
    %add smooth waypoints to the object.
    append(smoothWaypointsObj, smoothpath(:,1:3));
    
     % disp("distance...")
    distance= trajectory_distance(smoothpath);
    desired_distance= time_step*UAV_speed;
    nb_of_point= max(size(smoothpath,1),floor(distance /desired_distance));

    % disp("interpolate ...")
    interpolate(smoothWaypointsObj,nb_of_point)
    new_traj_UAV=smoothWaypointsObj.States(1:nb_of_point,:);

if (size(edited_cells,1)~=0)
    setOccupancy(map3D,edited_cells,0);
    disp("pseudo obstacles removed")
end

end

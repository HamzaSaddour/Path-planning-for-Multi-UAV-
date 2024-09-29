%demo
%loading the 3D occupancy map
addpath('Path_Planning_functions');
addpath('maps');
a=load("maps\ICL_inflated_10.mat");%a=load("maps\NYC_05_inflated10.mat"); % 
map3D=a.map3D_t;
map3D.OccupiedThreshold = 0.51; 
map3D.FreeThreshold = 0.5;
safety_distance=20;

%loading the UAV scenario
a=load("ICL_scene.mat");%a=load("maps\NYC_scene.mat");
scene=a.scene;

%selecting sart and goal poitions 
map_limits= [-300 300; -300 300; 50 200];

%uav number and start-goal positions 
starting_altitude=70;
start_goal = get_start_goal(scene,map3D,safety_distance);

adjusted_start_goal=start_goal;
for i = 1:size(start_goal,1)
    adjusted_start_goal(i,3)=max(adjusted_start_goal(i,3),starting_altitude);
end

colors =['red','m','b','y','c','g',"white","black"];%[57 -80 20 ; -25 -222 100],
nb_UAVs=floor(size(adjusted_start_goal,1)/2);%
UAVs_speed= 20*ones(nb_UAVs,1);%[2;2;2;2;2];
time_step= 0.01;%safety_distance/min(UAVs_speed);
uav_radius=10;

%planning initial paths for all the UAVs

tic

paths={};
distance={};

hold on
offset=0;
% tic
l=[];t=[];
tic
for i = 1:nb_UAVs
    
    view([0 90])
    sg=adjusted_start_goal(2*i-1:2*i,:);%%start_goal(2*i-1:2*i,:);
    disp("planning for UAV n° "+num2str(i))
    MaxConnectionDistance=20; %20 is better than 10
    min_goal_distance=5;
    nb_itirations=5;
    [path,ss,sv]=One_path_planning(sg,map3D,UAVs_speed(i),time_step,map_limits,MaxConnectionDistance,min_goal_distance,nb_itirations,2,0,0);%One_path_planning(start_goal,map3D,UAV_speed,time_step,map_limits,MaxConnectionDistance,min_goal_distance,nb_itirations,mode,fixed_wings)
    if (isempty(path))
        nb_UAVs=nb_UAVs-1;
        offset=offset+1;
        continue
    else
        paths{i-offset}=path;

    
    d = zeros(size(path,1)-1,1);
    for j = 2:size(path,1)
        d(j) = norm(path(j,1:3) - path(j-1,1:3));
    end
    distance{i} =d;
    l(i)=plot3(  path(:,1), ...
        path(:,2), ...
        path(:,3), ...
        "LineWidth",2,"Color",colors(mod(i,8)+1),'DisplayName', strcat('UAV ', int2str(i)) );
        % scatter3(path(:,1),path(:,2),path(:,3),'black','filled','o');
        t(1)=scatter3(path(end,1),path(end,2),path(end,3),'black','filled','o','DisplayName', 'Goal');
        t(2)=scatter3(path(1,1),path(1,2),path(1,3),'white','filled','o','MarkerEdgeColor','k','DisplayName', 'Start');
     end
end
legend([l(:); t(:)]);
planning_time=toc
%%
%plotting seperatio plot - Initial

waypoints = get_waypoints(paths);
numUAVs = size(waypoints, 2) / 3;
numTimeSteps = size(waypoints, 1);% Number of time steps
hold off
figure;
hold on;
for i = 1:numUAVs
    
    for j = i+1:numUAVs
        % Extract the coordinates for UAV i and UAV j
        n= min(size(paths{i},1),size(paths{j},1));
        uav_i_coords = waypoints(1:n, (i-1)*3 + 1:i*3);
        uav_j_coords = waypoints(1:n, (j-1)*3 + 1:j*3);
        
        % Calculate the Euclidean distance between UAV i and UAV j at each time step
        distances = sqrt(sum((uav_i_coords - uav_j_coords).^2, 2));
        % Plot the distances
        if (1)% min(distances)<50
            t=linspace(0,time_step*length(distances),length(distances))';
            plot(t,distances, 'LineWidth', 2,'DisplayName', sprintf('UAV %d & UAV %d', i, j));
        end
    end
end
yline(safety_distance, 'r', 'LineWidth', 2,'DisplayName', 'Safety Distance');
% Add labels and legend
xlabel('Time (s)');
ylabel('Distance (m)');

title('Distances Between Each Pair of UAVs - Initial Paths');
legend show;
ylim ([0 100]);
grid on;
hold off;

%editing paths 
%% 
waypoints = get_waypoints(paths);
e_paths=paths;
e_distance={};
ss = MyCustomStateSpace(map_limits);
edited_scene=copy(scene);
tic
for i = 1:nb_UAVs-1
    disp("editing UAV n° "+num2str(i))
    [e_paths{i},ss,sv]=edit_trajectory(i,map3D,UAVs_speed(i),time_step,edited_scene,e_paths,safety_distance);%(UAV,map3D,UAV_speed,time_step,scene,paths_,safety_distance)

    %uncomment to check the distance travelled during each time step
    % path=e_paths{i};
    % d = zeros(size(path,1)-1,1);
    % for j = 2:size(path,1)
    %     d(j) = norm(path(j,1:3) - path(j-1,1:3));
    % end
    % e_distance{i} =d;

end
editing_time = toc

all_over= planning_time+editing_time

%plotting results


figure(3);
show3D(scene);
view([0 90])
hold on

for i = 1:length(adjusted_start_goal)
    c=['w','k'];
    x=adjusted_start_goal(i,1);
    y=adjusted_start_goal(i,2);
    plot3(x, y, 100,'bo', 'MarkerFaceColor', c(mod(i,2)+1)); % Plot the point
    % Draw a semi-transparent red disk around the point
    radius = 10; % Set the radius of the disk
    theta = linspace(0, 2*pi, 100); % Parameter for the circle
    x_circle = radius * cos(theta) + x;
    y_circle = radius * sin(theta) + y;
    z=100*ones(1,length(y_circle));
    fill3(x_circle, y_circle,z, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

hold on

for i = 1:nb_UAVs
    path=e_paths{i};
    l(i)=plot3(  path(:,1), ...
        path(:,2), ...
        path(:,3), ...
        "LineWidth",2,"Color",colors(mod(i,8)+1),'DisplayName', strcat('UAV ', int2str(i)) );
end
legend([l(:)]);
 view([0 90])
 title('Final Paths');
%plotting results 2

figure(4);
show3D(edited_scene);%edited_scene
 view([0 90])
axis equal
hold on

for i = 1:nb_UAVs
    path=e_paths{i};
    l(i)=plot3(  path(:,1), ...
        path(:,2), ...
        path(:,3), ...
        "LineWidth",2,"Color",colors(mod(i,8)+1),'DisplayName', strcat('UAV ', int2str(i)) );
end
legend([l(:)]);
 view([0 90])
  title('Edited Paths with Pseudo-Obstacles');
%% plotting seperatio plot - Edited

waypoints = get_waypoints(e_paths);
numUAVs = size(waypoints, 2) / 3;
numTimeSteps = size(waypoints, 1);% Number of time steps

% Initialize a figure
figure(5);
hold on;
view([0 90])
for i = 1:numUAVs
    
    for j = i+1:numUAVs
        % Extract the coordinates for UAV i and UAV j
        n= min(size(e_paths{i},1),size(e_paths{j},1));
        uav_i_coords = waypoints(1:n, (i-1)*3 + 1:i*3);
        uav_j_coords = waypoints(1:n, (j-1)*3 + 1:j*3);
        
        % Calculate the Euclidean distance between UAV i and UAV j at each time step
        distances = sqrt(sum((uav_i_coords - uav_j_coords).^2, 2));
        % Plot the distances
        if (1) %min(distances)<50
            t=linspace(0,time_step*length(distances),length(distances))';
            plot(t,distances, 'LineWidth', 2,'DisplayName', sprintf('UAV %d & UAV %d', i, j));
        end
    end
end
yline(safety_distance, 'r', 'LineWidth', 2,'DisplayName', 'Safety Distance');
% Add labels and legend
xlabel('Time (s)');
ylabel('Distance (m)');

title('Distances Between Each Pair of UAVs - Edited Paths');
legend show;
ylim ([0 100]);
grid on;
hold off;

%%
for i = 1:5
    figure(i); % Create figure i
    % Set the position of the figure window (left, bottom, width, height)
    switch i
        case 1
            set(gcf, 'Position', [100, 500, 400, 300]); % Figure 1 position and size
        case 2
            set(gcf, 'Position', [500, 500, 600, 300]); % Figure 2
        case 3
            set(gcf, 'Position', [1100, 500, 400, 300]); % Figure 3
        case 4
            set(gcf, 'Position', [100, 100, 400, 300]); % Figure 4
        case 5
            set(gcf, 'Position', [600, 100, 600, 300]); % Figure 5
    end
end

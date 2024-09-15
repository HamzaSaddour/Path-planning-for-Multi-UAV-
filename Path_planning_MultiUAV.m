%Path planning for multiple UAV % takes over test_edit_trajectory
tic
%loading the map 
a=load("C:\Users\hs3223\Desktop\Project\code\maps\ICL_inflated_10.mat");%a=load("C:\Users\hs3223\Desktop\Project\code\maps\NYC_05_inflated10.mat"); % 

map3D=a.map3D_t;
map3D.OccupiedThreshold = 0.51; 
map3D.FreeThreshold = 0.5;

%selecting sart and goal poitions 
map_limits= [-300 300; -300 300; 50 200];
% figure;
% show3D(scene);
% view([0 90]);
% [x,y] = ginput ; %[xi,yi] = getpts(fig)
% z=100*ones(length(x),1);
% start_goal=[x,y,z]

% start_goal= {[-60 224 50; -58 -232 10], [-207 -218 30; 157 124 30], [-235 70 30; 185 -183 30],... 
%  [-15 120 110; -90 -223 30],[-222 -152 100; 76 148 30],[-293 270 50 ;186 -179 50], [7 296 30 ; -276 -201 30], [-45 -278 30; -297 164 25], [-282 22 30; 261 130 25], [-11 188 30;165 10 30]};
% adjusted_start_goal=start_goal;
% for i = 1:size(start_goal,2)
%     cell=adjusted_start_goal{i};
%     cell(1,3)=max(cell(1,3),100);
%     cell(2,3)=max(cell(2,3),100);
%     adjusted_start_goal{i}=cell;
% end
%%
a=load("C:\Users\hs3223\Desktop\Project\code\maps\ICL_scene.mat");%a=load("C:\Users\hs3223\Desktop\Project\code\maps\NYC_scene.mat");
scene=a.scene;
figure;
show3D(scene);
axis equal
% altitud=100;

%%
starting_altitude=70;
start_goal = get_start_goal(scene,map3D);

adjusted_start_goal=start_goal;
for i = 1:size(start_goal,1)
    adjusted_start_goal(i,3)=max(adjusted_start_goal(i,3),starting_altitude);
end
safety_distance=20;
colors =['red','m','b','y','c','g',"white","black"];%[57 -80 20 ; -25 -222 100],
nb_UAVs=floor(size(adjusted_start_goal,1)/2);%
UAVs_speed= 20*ones(nb_UAVs,1);%[2;2;2;2;2];
time_step= 0.01;%safety_distance/min(UAVs_speed);
uav_radius=10;

%%
% 
% adjusted_start_goal= [
%     -51.2191683393318,  -273.846650149399,  100;
%     139.994776888556,   206.062859442163,   100;
%     -119.956207473540,  257.303197705846,   100;
%     48.7619794922438,   -92.6308197046682,  100;
%     -268.678164873008,  16.0986785621701,   100;
%     187.485822108555,   168.569929005323,   100;
%     -222.436884000905,  -255.100184930978,  100;
%     38.7638647090861,   -142.621393620456,  100;
%     -164.947723997749,  -87.6317623130895,  100;
%     57.5103299275066,   71.0883098695365,   100;
% ];

% 
adjusted_start_goal= [ -60   224   100;
                       -58  -232   100;
                      -207  -218   100;
                       157   124   100;
                       185  -183   100;
                      -235    70   100;
                       -15   120   100;
                       -90  -223   100;
                        11 182 85;%5
                        193  186   80;%5
                      -293   270   100;
                       186  -179   100;
                         7   296   100;
                      -276  -201   100;
                       -45  -278   100;%8
                      -297   164   100;%8
                      -282    22   100;
                       261   130   100;
                       241   265   80;
                       26    -281   50];
nb_UAVs=floor(size(adjusted_start_goal,1)/2);
figure;
show3D(scene);

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
%%
%creating the scenario

% scene = uavScenario("ReferenceLocation",[(51.4970+51.5003)/2 (-0.1805-0.1705)/2 0]);%ICL --> [(51.4970+51.5003)/2 (-0.1805-0.1705)/2 0]
% xlimits=map_limits(1,:);
% ylimits=map_limits(2,:);
% zlimits=map_limits(3,:);
% 
% altitude = 0;
% color = [0.8 0.8 0.8];
% addMesh(scene,"terrain",{"gmted2010",xlimits,ylimits},color,Verbose=true);
% color = [0.7 1 0.7];
% addMesh(scene,"buildings",{"C:\Users\hs3223\Desktop\Project\code\maps\ICL_map.osm",xlimits,ylimits,"auto"},color,Verbose=true);%"ICL_map.osm"

figure;
show3D(scene);

% 
% scene = uavScenario("ReferenceLocation",[(40.7672+40.7528)/2 (-73.9859-73.9601)/2 0]);%ICL --> [(51.4970+51.5003)/2 (-0.1805-0.1705)/2 0]
% xlimits=map_limits(1,:);
% ylimits=map_limits(2,:);
% zlimits=map_limits(3,:);
% 
% altitude = 0;
% color = [0.8 0.8 0.8];
% addMesh(scene,"terrain",{"gmted2010",xlimits,ylimits},color,Verbose=true);
% color = [0.7 1 0.7];
% addMesh(scene,"buildings",{"C:\Users\hs3223\Desktop\Project\code\maps\NYC.osm",xlimits,ylimits,"auto"},color,Verbose=true);%"ICL_map.osm"
% 
% figure;
% show3D(scene);



% hold on   
% %         scatter3(adjusted_start_goal(7,1),adjusted_start_goal(7,2),adjusted_start_goal(7,3),'black','filled','o')
% %         scatter3(adjusted_start_goal(8,1),adjusted_start_goal(8,2),adjusted_start_goal(8,3),'white','filled','o')

%planning the paths for all the UAVs
%%
tic
figure;
show3D(scene);
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
% legend([l(:); t(:)]);
planning_time=toc

%%
% planning_time=toc
% 
waypoints = get_waypoints(paths);
e_paths=paths;
e_distance={};
ss = MyCustomStateSpace(map_limits);
edited_scene=copy(scene);
tic
for i = 1:nb_UAVs-1
    %copy_map=copy(map3D);
    disp("editing UAV n° "+num2str(i))
    [e_paths{i},ss,sv]=edit_trajectory(i,waypoints,ss,map3D,UAVs_speed(i),time_step,edited_scene,e_paths,safety_distance);%
    waypoints = get_waypoints(e_paths);

    path=e_paths{i};
    d = zeros(size(path,1)-1,1);
    for j = 2:size(path,1)
        d(j) = norm(path(j,1:3) - path(j-1,1:3));
    end
    e_distance{i} =d;

end
editing_time = toc

% all_over=toc

%%plotting results
%%
figure;
show3D(scene);%edited_scene

axis equal
hold on

for i = 1:nb_UAVs
    path=e_paths{i};
     % scatter3(path(:,1),path(:,2),path(:,3),'black','filled','o')
    % scatter3(path(1,1),path(1,2),path(1,3),'black','filled','+')
    l(i)=plot3(  path(:,1), ...
        path(:,2), ...
        path(:,3), ...
        "LineWidth",2,"Color",colors(mod(i,8)+1),'DisplayName', strcat('UAV ', int2str(i)) );
end
legend([l(:)]);
% hold on
% path=e_paths{2};
%     plot3(  path(:,1), ...
%         path(:,2), ...
%         path(:,3), ...
%         "LineWidth",2,"Color","y","LineStyle","--");
%%
% path=paths{1};
% hold on
% plot3(  path(:,1),path(:,2),path(:,3), "LineWidth",1,"Color",'y');
% %%
% figure;
% show3D(scene);
% hold on
% for i =1:9%1:nb_UAVs
%     path=paths{i};
%     plot3(  path(:,1),path(:,2),path(:,3), "LineWidth",2,"Color",colors(mod(i,5)+1));%[0.25 0.80 0.54]
% end
% %%
% path=paths{8};
% 
% hold on
% for i =1 : length(path)
%     scatter3(path(i,1),path(i,2),path(i,3),'black','filled')
% end
% %%
% hold on
% for i = -300:1:300
%     for j= -300:1:300
%         for z= 10:1:200
%             if (checkOccupancy(map3D,[i j z])==-1)
%                 scatter3(i,j,z,'red','filled')
%             end
%         end
%     end
% end
% 
% %%
% hold on
% a=[246.2247  148.5737  100.0000];
% scatter3(a(1), a(2), a(3),'cyan','filled')
% a=[-174.9458  149.8235  100.0000];
% scatter3(a(1), a(2), a(3),'blue','filled')
% 
%%
% Number of UAVs
paths_array=e_paths;
num_UAVs =size(paths_array, 2);

% Create a figure for the subplots
figure;
% i=[1 2 3 4 5 6]
for i = 1:num_UAVs
    % Get the current UAV's path
    uav_i_path = paths_array{i};
    
    % Number of time steps
    n = size(uav_i_path, 1);
    
    % Initialize an array to store distances to other UAVs
    distances = zeros(n, num_UAVs-1);
    
    % Calculate distances from UAV i to every other UAV
    k = 1; % Index for distances array
   subplot(5, 2, i);
   
   hold on
    for j = 1:num_UAVs

        if j ~= i
            uav_j_path = paths_array{j};
            n = min(size(uav_i_path, 1),size(uav_j_path, 1));

            uav_j_path=uav_j_path(1:n,:);
            uav_i_path=uav_i_path(1:n,:);
            
            % Calculate distance at each time step
            dist =vecnorm(uav_i_path(:,1:3) - uav_j_path(:,1:3), 2, 2);

            % sqrt((uav_i_path(:,1) - uav_j_path(:,1)).^2 + ...
            %             (uav_i_path(:,2) - uav_j_path(:,2)).^2);
            plot(dist);        
            % Store the distances in the array
            % distances(:, k) = dist;
            k = k + 1;
        end
    end
    
    % Create a subplot for the current UAV
  
    
    % Plot the distances
    % plot(1:n,distances);
    hold on;
    
    % Add a fixed line at distance 15
    yline(safety_distance, 'r', 'LineWidth', 1);
    
    % Title and labels
    title(['UAV ', num2str(i), ' Distance to Other UAVs']);
    xlabel('Time');
    ylabel('Distance');
     ylim([0, 200]);
    
    % Legend
    legend(arrayfun(@(x) ['UAV ', num2str(x)], setdiff(1:num_UAVs, i), 'UniformOutput', false), 'Location', 'Best');
    
    % Hold off to allow for multiple plots on the same subplot
    hold off;
end

% Adjust the layout to ensure subplots don't overlap
% tightfig;


%%
% Assuming 'paths' is the big matrix where every 3 columns correspond to the coordinates of a UAV
% For example: paths(:, 1:3) contains the coordinates for UAV 1, paths(:, 4:6) for UAV 2, etc.

% Number of UAVs
waypoints = get_waypoints(e_paths);

numUAVs = size(waypoints, 2) / 3;

% Number of time steps
numTimeSteps = size(waypoints, 1);

% Initialize a figure
figure;
hold on;
% i=[1 2 3 4 6 7]
% Loop through each pair of UAVs
for i = 1:numUAVs
    
    for j = i+1:numUAVs
         % if i==5 || j==5
         %    continue
         % end
        % Extract the coordinates for UAV i and UAV j
        uav_i_coords = waypoints(:, (i-1)*3 + 1:i*3);
        uav_j_coords = waypoints(:, (j-1)*3 + 1:j*3);
        
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
title('Distances Between Each Pair of UAVs');
legend show;
ylim ([0 100]);
grid on;
hold off;
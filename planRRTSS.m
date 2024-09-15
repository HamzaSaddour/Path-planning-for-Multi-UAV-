%RRT**
%this function will run RRT* algorithim multiple times with changing the
%random seed and get the best one that gives the shortest distance

function [pathObj, solnInfo,smoothed_path,distance,problem] = planRRTSS(planner,start,goal,nb_itirations,seed,ss,sv)
distance=inf;
distances=zeros(1,nb_itirations);
sols={};
problem=0;
for i=1:nb_itirations
    disp("iteration "+int2str(i))
    rng(i*seed)
    % tic
    [pthObj,solnInfo] = plan(planner,start,goal);
    % planning_time=toc
    sol.a=pthObj;
    sol.b=solnInfo;
    sol.c=pthObj.States;
    
    if (~solnInfo.IsPathFound)
        disp("No Path Found by the RRT")
        distances(i) = inf;
    else
       

        [smoothpath, problem]=PathSmoothing_star(ss,sv,pthObj.States);  
        % [smoothpath2, problem2]=PathSmoothing(ss,sv,pthObj.States);%MAthworks smoother
        distances(i)= trajectory_distance(smoothpath);
        sol.c=smoothpath;
        path =smoothpath;

        %% plot all possible pruned Paths
        % hold on
        % p1=plot3(  path(:,1), ...
        %         path(:,2), ...
        %         path(:,3), ...
        %         "LineWidth",1,"Color",'y','DisplayName', 'Possible Paths');
        % text(path(3,1),path(3,2),path(3,3),['   ',num2str(distances(i))])


        % distances(i) = solnInfo.PathCosts(end);
    end
    
    sols{i}=sol;

end

[distance,i]=min(distances);

opt=sols{i};
pathObj=opt.a;
solnInfo=opt.b;
smoothed_path=opt.c;

% smoothpath2=PathSmoothing_star(ss,sv,flipud(pthObj.States));
% path =smoothpath2;
        % hold on
        % plot3(  path(:,1), ...
        %         path(:,2), ...
        %         path(:,3), ...
        %         "LineWidth",2,"Color",'r');
       

% distance2= trajectory_distance(smoothpath2);
% if length(smoothpath2)<length(smoothed_path) % 2distance2<distance
%     smoothed_path=smoothpath2;
% end

% hold on
% path =pthObj.States;
%         hold on
%         p1=plot3(  path(:,1), ...
%                 path(:,2), ...
%                 path(:,3), ...
%                 "LineWidth",2,"Color",'y','DisplayName', 'Initial Path');
% 
% hold on
% path =smoothed_path;
% p3=plot3(  path(:,1), ...
%         path(:,2), ...
%         path(:,3),"LineWidth",2,"Color",'r','DisplayName', 'The Shortest Path');%, '-*'

% 
% 
%         path =smoothpath2;
%         hold on
%         p2=plot3(  path(:,1), ...
%                 path(:,2), ...
%                 path(:,3),'-*',"LineWidth",2,"Color",'b','DisplayName', 'Pruned Path- MATLAB function output');
% legend([p1 p3])%p1,p2,


%% plotting the the minimum distance as f of nb of iterations 
% 
% figure;
% 
% % Calculate the minimum distance at each repetition
% min_distances = cummin(distances);
% 
% % Create the plot
% plot(1:length(min_distances), min_distances, '-o', 'Color', 'b', 'MarkerFaceColor', 'b');
% 
% % Add labels and title
% title('Minimum Distance as a Function of Number of Repetitions');
% xlabel('Number of Repetitions');
% ylabel('Minimum Distance (m)');
% 
% % Add grid
% grid on;


end



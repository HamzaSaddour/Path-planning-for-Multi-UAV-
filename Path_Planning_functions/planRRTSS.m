%RRT**
%RRT double star
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
    [pthObj,solnInfo] = plan(planner,start,goal);
    sol.a=pthObj;
    sol.b=solnInfo;
    sol.c=pthObj.States;
    
    if (~solnInfo.IsPathFound)
        disp("No Path Found by the RRT")
        distances(i) = inf;
    else
        [smoothpath, problem]=PathSmoothing_star(sv,pthObj.States);  
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



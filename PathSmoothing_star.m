function [smoothpath,problem] = PathSmoothing_star(ss,sv,path)
smoothpath=path;
problem=0;
nonSmoothWaypoints=path;


startNode=1;
endNode=size(nonSmoothWaypoints,1);
counter=2;
lastNonCollisionNode=endNode;

optimizedWaypoints(startNode,:)=nonSmoothWaypoints(startNode,:); %take first state

if (size(nonSmoothWaypoints,1)==2)
    return
end
p=[];
for i = 1:size(nonSmoothWaypoints,1)-1
    MotionValid=isMotionValid(sv,nonSmoothWaypoints(i,:),nonSmoothWaypoints(i+1,:));
    if (~MotionValid)
        problem=1
        pt=nonSmoothWaypoints(i,:);
        p=[p;pt];
        scatter3(pt(1),pt(2),pt(3),'red','filled')

        
    end
end
if (problem)
    problem=1;
    smoothpath=path;
    p
    return 
end
hold on

while(endNode ~= startNode)
    % startNode
    % lastNonCollisionNode
    MotionValid=isMotionValid(sv,nonSmoothWaypoints(startNode,:),nonSmoothWaypoints(lastNonCollisionNode,:));
    % a=(nonSmoothWaypoints(lastNonCollisionNode,:)==[-128.0465 -7.1605 80.0946]);
    % if (a)
    %     disp("checking motion btn")
    %     nonSmoothWaypoints(startNode,:)
    %     nonSmoothWaypoints(lastNonCollisionNode,:)
    %     MotionValid
    % end

    if(MotionValid)
        optimizedWaypoints(counter,:)=nonSmoothWaypoints(lastNonCollisionNode,:);
        counter=counter+1;
        startNode=lastNonCollisionNode;
        % pt =nonSmoothWaypoints(lastNonCollisionNode,:);
        % scatter3(pt(1),pt(2),pt(3),'red','filled')
        lastNonCollisionNode=endNode;
    end
    if(~MotionValid)
        lastNonCollisionNode=lastNonCollisionNode-1;


    end
    
end
smoothpath= optimizedWaypoints(:,:);

end

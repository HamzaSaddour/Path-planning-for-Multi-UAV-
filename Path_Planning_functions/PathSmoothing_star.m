%This function proposes a new method for efficiently pruning the path
function [smoothpath,problem] = PathSmoothing_star(sv,path)
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

%verifying that the path is collision-free before prunning
for i = 1:size(nonSmoothWaypoints,1)-1
    MotionValid=isMotionValid(sv,nonSmoothWaypoints(i,:),nonSmoothWaypoints(i+1,:));
    if (~MotionValid)
        problem=1;
        pt=nonSmoothWaypoints(i,:);
        p=[p;pt];
        % scatter3(pt(1),pt(2),pt(3),'red','filled')

        
    end
end
if (problem)
    problem=1;
    %p conatains points with problems 
    smoothpath=path;
    return 
end

%the pruning algorithm 
while(endNode ~= startNode)
    MotionValid=isMotionValid(sv,nonSmoothWaypoints(startNode,:),nonSmoothWaypoints(lastNonCollisionNode,:));

    if(MotionValid)
        optimizedWaypoints(counter,:)=nonSmoothWaypoints(lastNonCollisionNode,:);
        counter=counter+1;
        startNode=lastNonCollisionNode;
        lastNonCollisionNode=endNode;
    end
    if(~MotionValid)
        lastNonCollisionNode=lastNonCollisionNode-1;
    end
    
end
smoothpath= optimizedWaypoints(:,:);

end

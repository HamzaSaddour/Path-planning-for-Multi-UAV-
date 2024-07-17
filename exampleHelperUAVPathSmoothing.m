function [smoothWaypointsObj] = exampleHelperUAVPathSmoothing(ss,sv,pthObj)
%exampleHelperUavPathSmoothing Smooth Dubins path iteratively

%   Copyright 2019 The MathWorks, Inc.

%removes intermediate waypoints between two distant poses
%if this does not result in a collision with the environment.

nonSmoothWaypoints=pthObj.States(:,1:3);
counter=1;
optimizedWaypoints(counter,:)=nonSmoothWaypoints(1,:); %take first state
startNode=1;
endNode=startNode+1;
counter=counter+1;
lastNonCollisionNode=endNode;
while(endNode<=length(nonSmoothWaypoints))
    MotionValid=isMotionValid(sv,nonSmoothWaypoints(startNode,:),nonSmoothWaypoints(endNode,:));
    collide=~MotionValid;
    if(~collide)
        optimizedWaypoints(counter,:)=nonSmoothWaypoints(endNode,:);
        lastNonCollisionNode=endNode;
        endNode=endNode+1;
    end
    if(collide)
        optimizedWaypoints(counter,:)=nonSmoothWaypoints(lastNonCollisionNode,:);
        counter=counter+1;
        startNode=lastNonCollisionNode;
        endNode=startNode+1;
    end
end
%define an empty navPath Object.
smoothWaypointsObj=navPath(ss);
%add smooth waypoints to the object.
append(smoothWaypointsObj, optimizedWaypoints(:,1:3));

end
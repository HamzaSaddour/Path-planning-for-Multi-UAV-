function distance = trajectory_distance(trajet)
waypoints = trajet.States;
nWayPoints = trajet.NumStates;

% Calculate the distance between waypoints
distance = 0;
for i = 2:nWayPoints
    distance= distance + norm(waypoints(i,1:3) - waypoints(i-1,1:3));
end

end
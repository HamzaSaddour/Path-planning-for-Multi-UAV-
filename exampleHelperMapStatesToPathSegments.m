function wp_indices = exampleHelperMapStatesToPathSegments(waypoints,states)
    nstates = size(states,1);
    nWayPoints = size(waypoints,1);
    wpdiff = zeros(nWayPoints,nstates);
    for i = 1:nWayPoints
        for j=1:nstates
            wpdiff(i,j) = norm(waypoints(i,:)-states(j,:));
        end
    end
    
    [~,minIndex] = min(wpdiff,[],2);
    wp_indices = zeros(nstates,1);
    
    wp_states = zeros(nWayPoints,7);
    for i = 1:nWayPoints-1
        wp_states(i,:) = states(minIndex(i),:);
        wp_indices(minIndex(i):minIndex(i+1)) = i;
    end

end


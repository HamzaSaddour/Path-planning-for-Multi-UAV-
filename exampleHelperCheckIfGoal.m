function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.5;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end
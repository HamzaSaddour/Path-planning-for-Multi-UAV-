%random start and goal positions generator
function start_goal_pairs = generate_random_positions(map_limits, n)
    % Extract limits for x, y, and z dimensions
    x_limits = map_limits(1, :);
    y_limits = map_limits(2, :);
    z_limits = map_limits(3, :);
    
    % Preallocate a cell array to hold the start-goal pairs
    start_goal_pairs = cell(1, n);
    
    for i = 1:n
        % Generate random start position
        start_x = x_limits(1) + (x_limits(2) - x_limits(1)) * rand();
        start_y = y_limits(1) + (y_limits(2) - y_limits(1)) * rand();
        start_z = z_limits(1) + (z_limits(2) - z_limits(1)) * rand();
        start_pos = [start_x, start_y, start_z];
        
        % Generate random goal position
        goal_x = x_limits(1) + (x_limits(2) - x_limits(1)) * rand();
        goal_y = y_limits(1) + (y_limits(2) - y_limits(1)) * rand();
        goal_z = z_limits(1) + (z_limits(2) - z_limits(1)) * rand();
        goal_pos = [goal_x, goal_y, goal_z];
        
        % Create a 2x3 matrix for the start-goal pair
        start_goal_pair = [start_pos; goal_pos];
        
        % Store the matrix in the cell array
        start_goal_pairs{i} = start_goal_pair;
    end
end


function start_goal = get_start_goal(scene,map)
figure
show3D(scene);
view([0 90]);
% altitude=starting_altitude;

prompt = {'Enter the number of UAVs:'}; % Define the prompt
dlg_title = 'Input'; % Title of the dialog box
num_lines = 1; % Number of lines for input
defaultans = {'1'}; % Default answer

% Open dialog box and get user input
answer = inputdlg(prompt, dlg_title, num_lines, defaultans);

% Convert the input from string to a number
N = str2double(answer{1});

% N = input('Enter the number of UAVs: ');

% Display a map
% figure;
% imshow('your_map_image.png'); % Replace 'your_map_image.png' with your actual map image file
hold on;
sg=[];
% Ask the user to click on N points
for i = 1:2*N
    c=['w','k'];
    got_point=0;
    while ~got_point

        [x, y] = ginput(1); % Get one click
        altitude = terrainHeight(scene,x,y)+10;
        count=0;
        while (checkOccupancy(map,[x y altitude]) && count<10)
            altitude=altitude+10;
            count=count+1
        end
        if (count == 10 || checkOccupancy(map,[x y altitude])==1)
            disp("This point is occupied, please select another one")
        else
            got_point=1;
        end
    end
    

    % z = terrainHeight(scene,x,y);
    plot3(x, y, altitude,'bo', 'MarkerFaceColor', c(mod(i,2)+1)); % Plot the point
    sg = [sg;x y altitude+5];
    % Draw a semi-transparent red disk around the point
    radius = 10; % Set the radius of the disk
    theta = linspace(0, 2*pi, 100); % Parameter for the circle
    x_circle = radius * cos(theta) + x;
    y_circle = radius * sin(theta) + y;
    z2=altitude*ones(1,length(y_circle));
    fill3(x_circle, y_circle,z2, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

    % altitude=starting_altitude;

end
% z=altitud*ones(length(x),1);
% start_goal=[sg,z];
start_goal=sg;
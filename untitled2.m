% close all
% close all hidden 
% clear all
%%
simTime = 1000;    % in seconds
updateRate = 1;  % in Hz
reference_location =[(40.7672+40.7528)/2 (-73.9859-73.9601)/2 0];
scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"ReferenceLocation",[(40.7672+40.7528)/2 (-73.9859-73.9601)/2 0]); % It should be set to within the latitude longitude ranges shown in the verbose output
%Add an inertial frame called MAP to the scenario.
%scene.addInertialFrame("ENU","MAP",trvec2tform([1 0 0])); %why this step ?
xlimits = [-300 300];
ylimits = [-300 300];
altitude = 0;
color = [0.6 0.6 0.6];
addMesh(scene,"terrain",{"gmted2010",xlimits,ylimits},color,Verbose=true)
color = [0 1 0];
osmInfo = addMesh(scene,"buildings",{"NYC.osm",xlimits,ylimits,"auto"},color,Verbose=true); %"buildings"	{osmFile,xBound,yBound,altitude}
%map is got from : https://www.openstreetmap.org/export#map=17/51.49870/-0.17547
%OSM : OpenStreetMap

% a=load("C:\Users\hs3223\Desktop\Project\code\maps\NYC_HQ.mat");
% map3D=a.map3D;
% map3D.FreeThreshold = 0.1;
% map3D.OccupiedThreshold = 0.1; 
%% ICL

simTime = 1000;    % in seconds
updateRate = 1;  % in Hz

reference_location =[(51.4970+51.5003)/2 (-0.1805-0.1705)/2 0];
scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"ReferenceLocation",[(51.4970+51.5003)/2 (-0.1805-0.1705)/2 0]); % It should be set to within the latitude longitude ranges shown in the verbose output
%Add an inertial frame called MAP to the scenario.
%scene.addInertialFrame("ENU","MAP",trvec2tform([1 0 0])); %why this step ?
xlimits = [-300 300];
ylimits = [-300 300];
altitude = 0;
color = [0.6 0.6 0.6];
addMesh(scene,"terrain",{"gmted2010",xlimits,ylimits},color,Verbose=true)
color = [0 1 0];
osmInfo = addMesh(scene,"buildings",{"ICL_map.osm",xlimits,ylimits,"auto"},color,Verbose=true); %"buildings"	{osmFile,xBound,yBound,altitude}
%map is got from : https://www.openstreetmap.org/export#map=17/51.49870/-0.17547
%OSM : OpenStreetMap

% a=load("C:\Users\hs3223\Desktop\Project\code\maps\NYC_HQ.mat");
% map3D=a.map3D;
% map3D.FreeThreshold = 0.1;
% map3D.OccupiedThreshold = 0.1; 
%%
figure;
show3D(scene);
%%
vertices=[xlimits(1)+50, ylimits(1)+50;xlimits(1)+50, ylimits(2)-50;xlimits(2)-50, ylimits(2)-50;xlimits(2)-50, ylimits(1)+50];
cs = uavCoverageSpace(Polygons={vertices},UseLocalCoordinates=true,ReferenceLocation=[0 0 0]); %Create the coverage space 
ReferenceHeight = 25;
cs.UnitWidth = 100;
cp = uavCoveragePlanner(cs,Solver="Exhaustive"); %create coverage planner
hold on;
show(cs);
takeoff = [300 -300 100];
[wp,soln] = plan(cp,takeoff);
hold on
plot3(wp(:,1),wp(:,2),wp(:,3),LineWidth=2);
% scatter3(wp(:,1),wp(:,2),wp(:,3),"red","filled");
plot(takeoff(1),takeoff(2),MarkerSize=25,Marker=".")
% legend("","","Path","Takeoff/Landing")

%%
x = [-300, -300, -300 , 0 , 300, 300, 300,0, -300,-200, -200, -200 , 0 , 200, 200, 200,0, -200,-100, -100, -100 , 0 , 100, 100, 100,0, -100,0,0,50,-50];
y = [-300, 0 , 300, 300 , 300, 0, -300,-300,-300,-200, 0 , 200, 200 , 200, 0, -200,-200,-200,-100, 0 , 100, 100 , 100, 0, -100,-100,-100,0,50,50,-50]; 

% x =[-6, -6, -114, -215, -224, -224, -237, -237, -237, -232, -122, -42, 18, 109, 159, 249, 255, 249, 249, 249, 258, 258, 258, 247, 142, 148, 150, 150, 150, 148, 159, 58, -21, -117, -129, -126, -126, -117, -25, 54, 65, 82, 75, 32, -21, -53, -53, 0];
% 
% y =[-3, 214, 211, 214, 123, 82, -12, -75, -149, -232, -251, -251, -251, -256, -256, -256, -182, -111, -58, -9, 49, 120, 164, 222, 239, 148, 82, 8, -50, -116, -185, -187, -187, -187, -129, -76, 59, 118, 96, 118, 32, -57, -121, -128, -128, -132, -22, -57];

x = [-306, -185, -67, 42, 107, 172, 279, 280, 284, 284, 286, 280, 275, 280, 228, 168, 106, 38, -34, -103, -203, -269, -297, -290, -286, -287, -287, -239, -171, -41, 48, 156, 232, 228, 218, 221, 226, 221, 132, 31, -35, -106, -236, -266, -251, -250, -243, -204, -120, -26, 129, 161, 161, 154, 158, 1, -100, -212, -233, -219, -218, -212, -165, -102, -8, 88, 84, 85, 93, 93, 2, -88, -192, -182, -174, -114, -8, 31, 31, 38, -34, -160, -149, -63, -2, -17, -28, -136, -142, -96];

y = [296, 299, 299, 296, 296, 296, 298, 173, 87, 0, -76, -156, -216, -288, -285, -286, -279, -279, -278, -274, -274, -272, -144, 0, 94, 213, 244, 245, 242, 242, 235, 238, 235, 155, 40, -83, -153, -205, -221, -220, -216, -216, -203, -108, 43, 143, 208, 206, 198, 195, 180, 79, 0, -80, -180, -173, -162, -156, -72, 61, 123, 155, 155, 140, 136, 130, 47, -25, -101, -137, -126, -109, -97, 76, 94, 96, 96, 100, 13, -90, -65, -54, 69, 60, 51, -62, -14, -7, 33, 25];

% x =[-300, -300, -200];
% y =[300, 200, 200] ;
z = 300*ones(size(wp,1),1);
waypoints =[wp(:,1:2),z];%[x' y' z']; 
%Specify the orientation as Euler angles (in radians) and convert the Euler angles to a quaternion:
orientation_eul = [0 0 0];
orientation_quat = quaternion(eul2quat(orientation_eul)); 
orientation_vec = repmat(orientation_quat,size(waypoints,1),1);
%Specify time vector
 time = 0:(simTime/(size(waypoints,1)-1)):simTime;
%Generate trajectory from the specified waypoints and orientations using waypointTrajectory system object. Specify the reference frame as 'ENU'. 
trajectory = waypointTrajectory("Waypoints",waypoints,"Orientation",orientation_vec, ...
    "SampleRate",updateRate,"ReferenceFrame","ENU","TimeOfArrival",time); 
%Specify the initial pose of the UAV
initial_pose = [0 0 300 1 0 0 0]; 
%Create a uavPlatform object and update the scenario with a mesh of the UAV. 
plat = uavPlatform("UAV2",scene,"Trajectory",trajectory,"ReferenceFrame","ENU"); 
updateMesh(plat,"quadrotor",{4},[1 0 0],eye(4)); %updateMesh(platform,type,geometries,color,position,orientation) %geo : "quadrotor"	{scale}

%Introduce a 3D lidar into the scenario using the uavLidarPointCloudGenerator system object. Specify relevant lidar sensor parameters. For example, the lidar in this example has a maximum range of 200 meters, but you can adjust the parameters as needed.
lidarmodel = uavLidarPointCloudGenerator("AzimuthResolution",0.1, ...
    "ElevationLimits",[-90 0],"ElevationResolution",1.25, ...
    "MaxRange",600,"UpdateRate",1,"HasOrganizedOutput",true);

lidar = uavSensor("Lidar2",plat,lidarmodel,"MountingLocation",[0 0 -1],"MountingAngles",[0 0 0]); 
%%
%Simulate Mapping Flight And Map Building
[ax,plotFrames] = show3D(scene);
xlim(xlimits);
ylim(ylimits);
zlim([0 200]);
view([-115 20]); 
axis equal 
hold on

colormap('jet');
ptc = pointCloud(nan(1,1,3));
scatterplot = scatter3(nan,nan,nan,1,[0.3020 0.7451 0.9333],"Parent",plotFrames.UAV2.Lidar2);
scatterplot.XDataSource = "reshape(ptc.Location(:,:,1), [], 1)";
scatterplot.YDataSource = "reshape(ptc.Location(:,:,2), [], 1)";
scatterplot.ZDataSource = "reshape(ptc.Location(:,:,3), [], 1)";
scatterplot.CDataSource = "reshape(ptc.Location(:,:,3), [], 1) - min(reshape(ptc.Location(:,:,3), [], 1))";
hold off; 

lidarSampleTime = [];
pt = cell(1,((updateRate*simTime) +1)); 
ptOut = cell(1,((updateRate*simTime) +1)); 
%Create an occupancy map for a more efficient way to store the point cloud data. Use a minimum resolution of 1 cell per meter.
% map3D = importOccupancyMap3D("nyc_test.ot"); %

map3D_t=occupancyMap3D(0.5);

% Simulate the mapping flight in the scenario. Store lidar sensor readings for each simulation step in a cell array after removing invalid points. Insert point clouds into the map using the insertPointCloud function. Ensure that the pose vector accounts for sensor offset. 
% restart(scene)
setup(scene); 

ptIdx = 0;
while scene.IsRunning
    ptIdx = ptIdx + 1;
    % Read the simulated lidar data from the scenario
    [isUpdated,lidarSampleTime,pt{ptIdx}] = read(lidar);

    if isUpdated
        % Get Lidar sensor's pose relative to ENU reference frame.
        sensorPose = getTransform(scene.TransformTree, "ENU","UAV2/Lidar2",lidarSampleTime);
        % Process the simulated Lidar pointcloud.
        ptc = pt{ptIdx};
        ptOut{ptIdx} = removeInvalidPoints(pt{ptIdx});
        % Construct the occupancy map using Lidar readings.
        insertPointCloud(map3D_t,[sensorPose(1:3,4)' tform2quat(sensorPose)],ptOut{ptIdx},500);

        % figure(1)
        % show3D(scene,"Time",lidarSampleTime,"FastUpdate",true,"Parent",ax);
        % xlim(xlimits);
        % ylim(ylimits);
        % zlim([0 200]);
        % view([0 90]);
        
        refreshdata
        drawnow limitrate
    end
    
    % Show map building real time 
    % figure(2)
    % show(map3D);
    % view([-115 20]);
    % axis equal 
    
    advance(scene);
    updateSensors(scene); 
    
end
map3D_t.OccupiedThreshold = 0.51; 
map3D_t.FreeThreshold = 0.5;
%%
inflate(map3D_t,10)
 figure
show(map3D_t)

%%
a=load("C:\Users\hs3223\Desktop\Project\code\scenario\map3D.mat");
map3D2=a.map3D;
inflate(map3D2,10)

figure
show(map3D2);
%%

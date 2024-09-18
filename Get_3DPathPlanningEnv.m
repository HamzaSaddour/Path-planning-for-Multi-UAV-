%Get_3DPathPlanningEnv

% The Get_3DPathPlanningEnv function takes as inputs the OpenStreetMap
% data, the dimensions of the area and its geographical coordinates’ centre. As output, the function 
% provides both the 3D occupancy map and the 3D UAV scenario, which can be used to implement
% the proposed multi-UAV path planning algorithm.

%coordinates : [Latitude;Longitude]
% map_limits= [xlimits;ylimits;zlimits] in meters, example :xlimits = [-300 300];
%map is got from : https://www.openstreetmap.org/export#map=17/51.49870/-0.17547
%OSM : OpenStreetMap
function [scene,OccMap]=Get_3DPathPlanningEnv(map_coordinates, map_limits,OpenStreetMap)
% 
% close all
% close all hidden 
% clear all
%%
simTime = 10;    % in seconds
updateRate = 1;  % in Hz

Latitude = map_coordinates(1,:);
Longitude = map_coordinates(2,:);
reference_location =[(Latitude(1)+Latitude(2))/2 , (Longitude(1)+Longitude(2))/2 ,0 ];
scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"ReferenceLocation",reference_location); % It should be set to within the latitude longitude ranges shown in the verbose output

xlimits = map_limits(1,:);
ylimits = map_limits(1,:);
altitude = 0;
color = [0.8 0.8 0.8];
addMesh(scene,"terrain",{"gmted2010",xlimits,ylimits},color,Verbose=true);
color = [0.7 1 0.7];
osmInfo = addMesh(scene,"buildings",{OpenStreetMap,xlimits,ylimits,"auto"},color,Verbose=true); %"buildings"	{osmFile,xBound,yBound,altitude}


figure;
show3D(scene);
view([0 90]);
%%
vertices=[xlimits(1)+50, ylimits(1)+50;xlimits(1)+50, ylimits(2)-50;xlimits(2)-50, ylimits(2)-50;xlimits(2)-50, ylimits(1)+50];
cs = uavCoverageSpace(Polygons={vertices},UseLocalCoordinates=true,ReferenceLocation=[0 0 0]); %Create the coverage space 
cs.UnitWidth = 100;
cp = uavCoveragePlanner(cs,Solver="Exhaustive"); %create coverage planner
hold on;
show(cs);
takeoff = [map_limits(1,1),map_limits(2,2),map_limits(3,2)];
[wp,soln] = plan(cp,takeoff);
hold on
plot3(wp(:,1),wp(:,2),wp(:,3),LineWidth=2);
plot(takeoff(1),takeoff(2),MarkerSize=25,Marker=".")

%%

z = map_limits(3,2)*ones(size(wp,1),1);
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

lidarSampleTime = [];
pt = cell(1,((updateRate*simTime) +1)); 
ptOut = cell(1,((updateRate*simTime) +1)); 
%Create an occupancy map for a more efficient way to store the point cloud data. Use a minimum resolution of 1 cell per meter.
OccMap=occupancyMap3D(0.5);

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
        insertPointCloud(OccMap,[sensorPose(1:3,4)' tform2quat(sensorPose)],ptOut{ptIdx},500);
        refreshdata
        drawnow limitrate
    end
   
    
    advance(scene);
    updateSensors(scene); 
    
end
OccMap.OccupiedThreshold = 0.51; 
OccMap.FreeThreshold = 0.5;
%%
inflate(OccMap,10)
 figure
show(OccMap)
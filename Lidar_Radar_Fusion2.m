dtedfile = "n39_w106_3arc_v2.dt1";
buildingfile = "ICL_map.osm";
scene = createScenario(dtedfile,buildingfile);

% Waypoints 
WP1=load("WP1.mat");
waypointsA=WP1.interpolatedSmoothWaypoints1.States;
timeA = linspace(0,41,size(waypointsA,1));
trajA = waypointTrajectory(waypointsA, "TimeOfArrival", timeA, "ReferenceFrame", "ENU", "AutoBank", true);
uavA = uavPlatform("UAV", scene, "Trajectory", trajA, "ReferenceFrame", "ENU");
updateMesh(uavA, "quadrotor", {5}, [0.6350    0.0780    0.1840], eye(4));

WP2=load("WP2.mat");
WP2=WP2.interpolatedSmoothWaypoints2.States;
waypointsC = WP2;
timeC = linspace(0,41,size(waypointsC,1));
trajC = waypointTrajectory(waypointsC, "TimeOfArrival", timeC, "ReferenceFrame", "ENU", "AutoBank", true);
uavC = uavPlatform("UAV3", scene, "Trajectory", trajC, "ReferenceFrame", "ENU");
updateMesh(uavC, "quadrotor", {5}, [0.6350    0.0780    0.1840], eye(4));

waypointsB = [194 120 50; 180 50 20];
timeB = [0 41];
trajB = waypointTrajectory(waypointsB, "TimeOfArrival", timeB, "ReferenceFrame", "ENU", "AutoBank", true);
uavB = uavPlatform("UAV2", scene, "Trajectory", trajB, "ReferenceFrame", "ENU");
updateMesh(uavB, "fixedwing", {10},  [0.6350    0.0780    0.1840], eye(4));

waypointsD = [0 0 200 ;0 0 200];
timeD = [0 41];
trajD = waypointTrajectory(waypointsD, "TimeOfArrival", timeD, ...
    "ReferenceFrame", "ENU", "AutoBank", true, "AutoPitch", true);
egoUAV = uavPlatform("EgoVehicle", scene, "Trajectory", trajD, "ReferenceFrame", "ENU");
updateMesh(egoUAV, "quadrotor", {5}, [0 0 1], eye(4));

% Mount a lidar on the quadrotor
lidarOrient = [90 90 0];
lidarSensor = uavLidarPointCloudGenerator("MaxRange",100, ...
    "RangeAccuracy", 0.03, ...
    "ElevationLimits", [-15 15], ...
    "ElevationResolution", 2, ...
    "AzimuthLimits", [-180 180], ...
    "AzimuthResolution", 0.2, ...
    "UpdateRate", 10, ...
    "HasOrganizedOutput", false);
lidar = uavSensor("Lidar", egoUAV, lidarSensor, "MountingLocation", [0 0 -3], "MountingAngles",lidarOrient);

% Mount a radar on the quadrotor
radarSensor = radarDataGenerator("no scanning","SensorIndex",1,...
    "FieldOfView",[120 80],...
    "UpdateRate", 1,...
    'MountingAngles',[0 30 0],...
    "HasElevation", true,...
    "ElevationResolution", 6,...
    "AzimuthResolution", 2, ...
    "RangeResolution", 4, ...
    "RangeLimits", [0 200],...
    'ReferenceRange',200,...
    'CenterFrequency',24.55e9,...
    'Bandwidth',200e6,...
    "TargetReportFormat","Detections",...
    "DetectionCoordinates","Sensor rectangular",...
    "HasFalseAlarms",false,...
    "FalseAlarmRate", 1e-7);

radar = uavSensor("Radar",egoUAV,helperRadarAdaptor(radarSensor));

lidarDetector = helperLidarDetector(scene)

lidarJPDA = trackerJPDA('TrackerIndex',2,...
    'AssignmentThreshold',[70 150],...
    'ClutterDensity',1e-16,...
    'DetectionProbability',0.99,...
    'DeletionThreshold',[10 10],... Delete lidar track if missed for 1 second
    'ConfirmationThreshold',[4 5],...
    'FilterInitializationFcn',@initLidarFilter);

radarPHD = createRadarTracker(radarSensor, egoUAV);
% Track Fusion
radarConfig = fuserSourceConfiguration('SourceIndex',1,...
    'IsInitializingCentralTracks',true);

lidarConfig = fuserSourceConfiguration('SourceIndex',2,...
    'IsInitializingCentralTracks',true);

fuser = trackFuser('SourceConfigurations',{radarConfig,lidarConfig},...
    'ProcessNoise',blkdiag(2*eye(6),1*eye(3),0.2*eye(4)),...
    'HasAdditiveProcessNoise',true,...
    'AssignmentThreshold',200,...
    'ConfirmationThreshold',[4 5],...
    'DeletionThreshold',[5 5],...
    'StateFusion','Cross',...
    'StateTransitionFcn',@augmentedConstvel,...
    'StateTransitionJacobianFcn',@augmentedConstvelJac);

viewer = helperUAVDisplay(scene);
%%
% Radar and lidar coverages for display
[radarcov,lidarcov] = sensorCoverage(radarSensor, lidar);
%% Simulation

setup(scene);
s = rng;
rng(2021);

numSteps = scene.StopTime*scene.UpdateRate;
truthlog = cell(1,numSteps);
radarlog = cell(1,numSteps);
lidarlog = cell(1,numSteps);
fusedlog = cell(1,numSteps);
logCount = 0;

while advance(scene)
    time = scene.CurrentTime;
    % Update sensor readings and read data.
    updateSensors(scene);
    egoPose = read(egoUAV);

    % Track with radar
    [radardets, radarTracks, inforadar] = updateRadarTracker(radar,radarPHD, egoPose, time);

    % Track with lidar
    [lidardets, lidarTracks, nonGroundCloud, groundCloud] = updateLidarTracker(lidar,lidarDetector, lidarJPDA, egoPose);

    % Fuse lidar and radar tracks
    rectRadarTracks = formatPHDTracks(radarTracks);
    if isLocked(fuser) || ~isempty(radarTracks) || ~isempty(lidarTracks)
        [fusedTracks,~,allfused,info] = fuser([lidarTracks;rectRadarTracks],time);
    else
        fusedTracks = objectTrack.empty;
    end

    % Save log
    logCount = logCount + 1;
    lidarlog{logCount} = lidarTracks;
    radarlog{logCount} = rectRadarTracks;
    fusedlog{logCount} = fusedTracks;
    truthlog{logCount} = logTargetTruth(scene.Platforms(1:3));
    
    % Update figure
    viewer(radarcov, lidarcov, nonGroundCloud, groundCloud, lidardets, radardets, lidarTracks, radarTracks, fusedTracks );
end

%% supporting functions 
% function scene = createScenario(buildingfile)
% % 
% % try
% %     addCustomTerrain("southboulder",dtedfile);
% % catch
% %     % custom terrain was already added.
% % end
% 
% simTime =60;    % in seconds
% updateRate = 10;  % in Hz
% scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"ReferenceLocation",[(51.4970+51.5003)/2 (-0.1805-0.1705)/2 0]); % It should be set to within the latitude longitude ranges shown in the verbose output
% 
% %minHeight = 1.6925e+03;
% %latlonCenter = [39.9786 -105.2882 minHeight];
% 
% % scene = uavScenario("UpdateRate",10,"StopTime",40,...
% %     "ReferenceLocation",latlonCenter);
% color = [0 0 0];
% % Add terrain mesh
% sceneXLim = [-300 300];
% sceneYLim = [-300 300];
% %scene.addMesh("terrain", {"southboulder", sceneXLim, sceneYLim},[0 0 0]);
% addMesh(scene,"terrain",{"gmted2010",sceneXLim,sceneYLim},color,Verbose=true)
% 
% % Add buildings
% scene.addMesh("buildings", {buildingfile, sceneXLim, sceneYLim, "auto"}, [0 0 0]);
% 
% end


function scene = createScenario(dtedfile,buildingfile)

try
    addCustomTerrain("ICL_map",dtedfile);
catch
    % custom terrain was already added.
end

minHeight = 1.6925e+03;
latlonCenter = [39.9786 -105.2882 minHeight];
scene = uavScenario("UpdateRate",10,"StopTime",40,...
    "ReferenceLocation",latlonCenter);

% Add terrain mesh
sceneXLim = [-300 300];
sceneYLim = [-300 300];
scene.addMesh("terrain", {"ICL_map", sceneXLim, sceneYLim},[0 0 0]);

% Add buildings
scene.addMesh("buildings", {buildingfile, sceneXLim, sceneYLim, "auto"}, [0 0 0]);
%scene.addMesh("buildings",{"ICL_map.osm",xlimits,ylimits,"auto"},color,Verbose=true);

end



%createRadarTracker creates the trackerPHD tracker to fuse radar detections.
function tracker = createRadarTracker(radar, egoUAV)

% Create sensor configuration for trackerPHD
fov = radar.FieldOfView;
sensorLimits = [-fov(1)/2 fov(1)/2; -fov(2)/2 fov(2)/2; 0 inf];
sensorResolution = [radar.AzimuthResolution;radar.ElevationResolution; radar.RangeResolution];
Kc = radar.FalseAlarmRate/(radar.AzimuthResolution*radar.RangeResolution*radar.ElevationResolution);
Pd = radar.DetectionProbability;

sensorPos = radar.MountingLocation(:);
sensorOrient = rotmat(quaternion(radar.MountingAngles, 'eulerd', 'ZYX', 'frame'),'frame');

% Specify frame info of radar with respect to UAV
sensorTransformParameters(1) = struct('Frame','Spherical',...
    'OriginPosition', sensorPos,...
    'OriginVelocity', zeros(3,1),...% Sensor does not move relative to ego
    'Orientation', sensorOrient,...
    'IsParentToChild',true,...% Frame rotation is supplied as orientation
    'HasElevation',true,...
    'HasVelocity',false);

% Specify frame info of UAV with respect to scene
egoPose = read(egoUAV);
sensorTransformParameters(2) = struct('Frame','Rectangular',...
    'OriginPosition', egoPose(1:3),...
    'OriginVelocity', egoPose(4:6),...
    'Orientation', rotmat(quaternion(egoPose(10:13)),'Frame'),...
    'IsParentToChild',true,...
    'HasElevation',true,...
    'HasVelocity',false);

radarPHDconfig = trackingSensorConfiguration(radar.SensorIndex,...
    'IsValidTime', true,...
    'SensorLimits',sensorLimits,...
    'SensorResolution', sensorResolution,...
    'DetectionProbability',Pd,...
    'ClutterDensity', Kc,...
    'SensorTransformFcn',@cvmeas,...
    'SensorTransformParameters', sensorTransformParameters);

radarPHDconfig.FilterInitializationFcn = @initRadarFilter;

radarPHDconfig.MinDetectionProbability = 0.4;

% Threshold for partitioning
threshold = [3 16];
tracker = trackerPHD('TrackerIndex',1,...
    'HasSensorConfigurationsInput',true,...
    'SensorConfigurations',{radarPHDconfig},...
    'BirthRate',1e-3,...
    'AssignmentThreshold',50,...% Minimum negative log-likelihood of a detection cell to add birth components
    'ExtractionThreshold',0.80,...% Weight threshold of a filter component to be declared a track
    'ConfirmationThreshold',0.99,...% Weight threshold of a filter component to be declared a confirmed track
    'MergingThreshold',50,...% Threshold to merge components
    'DeletionThreshold',0.1,...% Threshold to delete components
    'LabelingThresholds',[1.01 0.01 0],...% This translates to no track-splitting. Read LabelingThresholds help
    'PartitioningFcn',@(dets) partitionDetections(dets, threshold(1),threshold(2),'Distance','euclidean'));
end
%initRadarfilter implements the GGIW-PHD filter used by the trackerPHD object. This filter is used during a tracker update to initialize new birth components in the density and to initialize new components from detection partitions.
function phd = initRadarFilter (detectionPartition)

if nargin == 0

    % Process noise
    sigP = 0.2;
    sigV = 1;
    Q = diag([sigP, sigV, sigP, sigV, sigP, sigV].^2);

    phd = ggiwphd(zeros(6,0),repmat(eye(6),[1 1 0]),...
        'ScaleMatrices',zeros(3,3,0),...
        'MaxNumComponents',1000,...
        'ProcessNoise',Q,...
        'HasAdditiveProcessNoise',true,...
        'MeasurementFcn', @cvmeas,...
        'MeasurementJacobianFcn', @cvmeasjac,...
        'PositionIndex', [1 3 5],...
        'ExtentRotationFcn', @(x,dT)eye(3,class(x)),...
        'HasAdditiveMeasurementNoise', true,...
        'StateTransitionFcn', @constvel,...
        'StateTransitionJacobianFcn', @constveljac);

else %nargin == 1
    % ------------------
    % 1) Configure Gaussian mixture
    % 2) Configure Inverse Wishart mixture
    % 3) Configure Gamma mixture
    % -----------------

    %% 1) Configure Gaussian mixture
    meanDetection = detectionPartition{1};
    n = numel(detectionPartition);

    % Collect all measurements and measurement noises.
    allDets = [detectionPartition{:}];
    zAll = horzcat(allDets.Measurement);
    RAll = cat(3,allDets.MeasurementNoise);

    % Specify mean noise and measurement
    z = mean(zAll,2);
    R = mean(RAll,3);
    meanDetection.Measurement = z;
    meanDetection.MeasurementNoise = R;

    % Parse mean detection for position and velocity covariance.
    [posMeas,velMeas,posCov] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(meanDetection,'initRadarFilter','double');

    % Create a constant velocity state and covariance
    states = zeros(6,1);
    covariances = zeros(6,6);
    states(1:2:end) = posMeas;
    states(2:2:end) = velMeas;
    covariances(1:2:end,1:2:end) = posCov;
    covariances(2:2:end,2:2:end) = 10*eye(3);

    % process noise
    sigP = 0.2;
    sigV = 1;
    Q = diag([sigP, sigV, sigP, sigV, sigP, sigV].^2);

    %% 2) Configure Inverse Wishart mixture parameters
    % The extent is set to the spread of the measurements in positional-space.
    e = zAll - z;
    Z = e*e'/n + R;
    dof = 150;
    % Measurement Jacobian
    p = detectionPartition{1}.MeasurementParameters;
    H = cvmeasjac(states,p);

    Bk = H(:,1:2:end);
    Bk2 = eye(3)/Bk;
    V = (dof-4)*Bk2*Z*Bk2';

    % Configure Gamma mixture parameters such that the standard deviation
    % of the number of detections is n/4
    alpha = 16; % shape
    beta = 16/n; % rate

    phd = ggiwphd(...
        ... Gaussian parameters
        states,covariances,...
        'HasAdditiveMeasurementNoise' ,true,...
        'ProcessNoise',Q,...
        'HasAdditiveProcessNoise',true,...
        'MeasurementFcn', @cvmeas,...
        'MeasurementJacobianFcn' , @cvmeasjac,...
        'StateTransitionFcn', @constvel,...
        'StateTransitionJacobianFcn' , @constveljac,...
        'PositionIndex'  ,[1 3 5],...
        'ExtentRotationFcn' , @(x,dT) eye(3),...
        ... Inverse Wishart parameters
        'DegreesOfFreedom',dof,...
        'ScaleMatrices',V,...
        'TemporalDecay',150,...
        ... Gamma parameters
        'Shapes',alpha,'Rates',beta,...
        'GammaForgettingFactors',1.05);
end

end
%formatPHDTracks formats the elliptical GGIW-PHD tracks into rectangular augmented state tracks for track fusion. convertExtendedTrack returns state and state covariance of the augmented rectangular state. The Inverse Wishart random matrix eigen values are used to derive rectangular box dimensions. The eigen vectors provide the orientation quaternion. In this example, you use an arbitrary covariance for radar track dimension and orientation, which is often sufficient for tracking.
function tracksout = formatPHDTracks(tracksin)
% Convert track struct from ggiwphd to objectTrack with state definition
% [x y z vx vy vz L W H q0 q1 q2 q3]
N = numel(tracksin);
tracksout = repmat(objectTrack,N,1);
for i=1:N
    tracksout(i) = objectTrack(tracksin(i));
    [state, statecov] = convertExtendedTrack(tracksin(i));
    tracksout(i).State = state;
    tracksout(i).StateCovariance = statecov;
end
end

function [state, statecov] = convertExtendedTrack(track)
% Augment the state with the extent information

extent = track.Extent;
[V,D] = eig(extent);
% Choose L > W > H. Use 1.5 sigma as the dimension
[dims, idx] = sort(1.5*sqrt(diag(D)),'descend');
V = V(:,idx);
q = quaternion(V,'rotmat','frame');
q = q./norm(q);
[q1, q2, q3, q4] = parts(q);
state = [track.State; dims(:); q1 ; q2 ; q3 ; q4 ];
statecov = blkdiag(track.StateCovariance, 4*eye(3), 4*eye(4));

end
%updateRadarTracker updates the radar tracking chain. The function first reads the current radar returns. Then the radar returns are passed to the GGIW-PHD tracker after updating its sensor configuration with the current pose of the ego drone.
function [radardets, radarTracks, inforadar] = updateRadarTracker(radar,radarPHD, egoPose, time)
[~,~,radardets, ~, ~] = read(radar); % isUpdated and time outputs are not compatible with this workflow
inforadar = [];
if mod(time,1) ~= 0
    radardets = {};
end
if mod(time,1) == 0 && (isLocked(radarPHD) || ~isempty(radardets))
    % Update radar sensor configuration for the tracker
    configs = radarPHD.SensorConfigurations;
    configs{1}.SensorTransformParameters(2).OriginPosition = egoPose(1:3);
    configs{1}.SensorTransformParameters(2).OriginVelocity = egoPose(4:6);
    configs{1}.SensorTransformParameters(2).Orientation = rotmat(quaternion(egoPose(10:13)),'frame');
    [radarTracks,~,~,inforadar] = radarPHD(radardets,configs,time);
elseif isLocked(radarPHD)
    radarTracks = predictTracksToTime(radarPHD,'confirmed', time);
    radarTracks = arrayfun(@(x) setfield(x,'UpdateTime',time), radarTracks);
else
    radarTracks = objectTrack.empty;
end
end

%updateLidarTracker updates the lidar tracking chain. The function first reads the current point cloud output from the lidar sensor. Then the point cloud is processed to extract object detections. Finally, these detections are passed to the point target tracker.
function [lidardets, lidarTracks,nonGroundCloud, groundCloud] = updateLidarTracker(lidar, lidarDetector,lidarJPDA, egoPose)
[~, time, ptCloud] = read(lidar);
% lidar is always updated
[lidardets,nonGroundCloud, groundCloud] = lidarDetector(egoPose, ptCloud,time);
if isLocked(lidarJPDA) || ~isempty(lidardets)
    lidarTracks = lidarJPDA(lidardets,time);
else
    lidarTracks = objectTrack.empty;
end
end
%initLidarFilter initializes the filter for the lidar tracker. The initial track state is derived from the detection position measurement. Velocity is set to 0 with a large covariance to allow future detections to be associated to the track. Augmented state motion model, measurement functions, and Jacobians are also defined here.
function ekf = initLidarFilter(detection)

% Lidar measurement: [x y z L W H q0 q1 q2 q3]
meas = detection.Measurement;
initState = [meas(1);0;meas(2);0;meas(3);0; meas(4:6);meas(7:10) ];
initStateCovariance = blkdiag(100*eye(6), 100*eye(3), eye(4));

% Process noise standard deviations
sigP = 1;
sigV = 2;
sigD = 0.5; % Dimensions are constant but partially observed
sigQ = 0.5;

Q = diag([sigP, sigV, sigP, sigV, sigP, sigV, sigD, sigD, sigD, sigQ, sigQ, sigQ, sigQ].^2);

ekf = trackingEKF('State',initState,...
    'StateCovariance',initStateCovariance,...
    'ProcessNoise',Q,...
    'StateTransitionFcn',@augmentedConstvel,...
    'StateTransitionJacobianFcn',@augmentedConstvelJac,...
    'MeasurementFcn',@augmentedCVmeas,...
    'MeasurementJacobianFcn',@augmentedCVmeasJac);
end

function stateOut = augmentedConstvel(state, dt)
% Augmented state for constant velocity
stateOut = constvel(state(1:6,:),dt);
stateOut = vertcat(stateOut,state(7:end,:));
% Normalize quaternion in the prediction stage
idx = 10:13;
qparts = stateOut(idx,:);
n = sqrt(sum(qparts.^2));
qparts = qparts./n;
stateOut(idx,qparts(1,:)<0) = -qparts(:,qparts(1,:)<0);
end

function jacobian = augmentedConstvelJac(state,varargin)
jacobian = constveljac(state(1:6,:),varargin{:});
jacobian = blkdiag(jacobian, eye(7));
end

function measurements = augmentedCVmeas(state)
measurements = cvmeas(state(1:6,:));
measurements = [measurements; state(7:9,:); state(10:13,:)];
end

function jacobian = augmentedCVmeasJac(state,varargin)
jacobian = cvmeasjac(state(1:6,:),varargin{:});
jacobian = blkdiag(jacobian, eye(7));
end
%sensorCoverage constructs sensor coverage configuration structures for visualization.
function [radarcov,lidarcov] = sensorCoverage(radarSensor, lidar)
radarcov = coverageConfig(radarSensor);
% Scale down coverage to limit visual clutter
radarcov.Range = 10;
lidarSensor = lidar.SensorModel;
lidarcov = radarcov;
lidarcov.Index = 2;
lidarcov.FieldOfView = [diff(lidarSensor.AzimuthLimits); diff(lidarSensor.ElevationLimits)];
lidarcov.Range = 5;
lidarcov.Orientation = quaternion(lidar.MountingAngles,'eulerd','ZYX','frame');
end
%logTargetTruth logs true pose and dimensions throughout the simulation for performance analysis.
function logEntry = logTargetTruth(targets)
n = numel(targets);
targetPoses = repmat(struct('Position',[],'Velocity',[],'Dimension',[],'Orientation',[]),1,n);
uavDimensions = [5 5 0.3 ; 9.8 8.8 2.8; 5 5 0.3];
for i=1:n
    pose = read(targets(i));
    targetPoses(i).Position = pose(1:3);
    targetPoses(i).Velocity = pose(4:6);
    targetPoses(i).Dimension = uavDimensions(i,:);
    targetPoses(i).Orientation = pose(10:13);
    targetPoses(i).PlatformID = i;
end
logEntry = targetPoses;
end
%metricDistance defines a custom distance for GOSPA. This distance incorporates errors in position, velocity, dimension, and orientation of the tracks.
function out = metricDistance(track,truth)
positionIdx = [1 3 5];
velIdx = [2 4 6];
dimIdx = 7:9;
qIdx = 10:13;

trackpos = track.State(positionIdx);
trackvel = track.State(velIdx);
trackdim = track.State(dimIdx);
trackq = quaternion(track.State(qIdx)');

truepos = truth.Position;
truevel = truth.Velocity;
truedim = truth.Dimension;
trueq = quaternion(truth.Orientation);

errpos = truepos(:) - trackpos(:);
errvel = truevel(:) - trackvel(:);
errdim = truedim(:) - trackdim(:);

% Weights expressed as inverse of the desired accuracy
posw = 1/0.2; %m^-1
velw = 1/2; % (m/s) ^-1
dimw = 1/4; % m^-1
orw = 1/20; % deg^-1

distPos = sqrt(errpos'*errpos);
distVel = sqrt(errvel'*errvel);
distdim = sqrt(errdim'*errdim);
distq = rad2deg(dist(trackq, trueq));

out = (distPos * posw + distVel * velw + distdim * dimw + distq * orw)/(posw + velw + dimw + orw);
end
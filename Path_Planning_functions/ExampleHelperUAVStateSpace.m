classdef ExampleHelperUAVStateSpace < nav.StateSpace & ...
        matlabshared.planning.internal.EnforceScalarHandle
    %ExampleHelperUAVStateSpace State space that connects with 3D Dubin's path
    %   The ExampleHelperUAVStateSpace constructs a state space that
    %   includes [x, y, z, psi], where [x, y, z] is the UAV's position
    %   in NED frame and psi is the UAV's heading angle. Each pair of state
    %   are connected using uavDubinsConnection, which is more suitable for
    %   fixed-wing UAV to follow.
    %
    %   This class is based on the source code template generated by
    %   createPlanningTemplate("StateSpace")
    %
    %   See also: uavDubinsConnection, createPlanningTemplate
    
    %   Copyright 2019-2020 The MathWorks, Inc.
    
    properties
        %PWRGuidedSampling Bias towards sampling point in goal region
        %   Higher value let planner to sample more points near the goal
        PWGRGuidedSampling=0;
    end
    
    properties (SetAccess = protected)
        %UniformDistribution - Uniform distribution for sampling
        UniformDistribution
        
        %NormalDistribution - Normal distribution for sampling
        NormalDistribution
        
        %WorkspaceGoalRegion Bounding box for goal region
        WorkspaceGoalRegion
        
        %DubinsPathSettings Dubin Path Settings for path planning
        DubinsPathSettings
        
        %AirSpeed Air speed configuration used in the state space
        AirSpeed
    end
    
    properties (Constant, Access = protected)
        %DefaultMaxRollAngle Default values for Max Roll Angle
        DefaultMaxRollAngle=pi/6
        
        %DefaultAirSpeed Default values for AirSpeed
        DefaultAirSpeed=20
        
        %DefaultFlightPathAngleLimit Default values for FlightPathAngleLimit
        DefaultFlightPathAngleLimit= [-0.1 0.1]
        
        %DefaultRandomStateBounds Default values for State Bounds
        DefaultRandomStateBounds = ...
            [-100, 100; ...
            -100, 100; ...
            -100 100; ...
            -pi,  pi]

         % [-100, 100; ...
         %    -100, 100; ...
         %    -100 100; ...
         %    -pi,  pi]
         % 
    end
    
    methods
        function obj = ExampleHelperUAVStateSpace(varargin)
            %State Space Constructor for ExampleHelperUAVStateSpace object
            
            name = 'ExampleHelperUAVStateSpace';
            numStateVariables = 4;
            
            % Call the constructor of the base class
            obj@nav.StateSpace(name, numStateVariables, ExampleHelperUAVStateSpace.DefaultRandomStateBounds);
            
            % Create the probability distributions for sampling.
            obj.NormalDistribution = matlabshared.tracking.internal.NormalDistribution(numStateVariables);
            obj.UniformDistribution = matlabshared.tracking.internal.UniformDistribution(numStateVariables);
            
            if nargin ~= 0
                [mxRoll,arspeed,flightpathangle,bounds]=ExampleHelperUAVStateSpace.parseStateSpaceConstuctor(varargin{:});
                
            end
            if (nargin ==0)
                mxRoll=obj.DefaultMaxRollAngle;
                arspeed=obj.DefaultAirSpeed;
                flightpathangle=obj.DefaultFlightPathAngleLimit;
                bounds=obj.DefaultRandomStateBounds;
            end
            obj.AirSpeed = arspeed;
            obj.DubinsPathSettings=uavDubinsConnection('MaxRollAngle',mxRoll,'AirSpeed',arspeed,'FlightPathAngleLimit',flightpathangle);
            obj.StateBounds = bounds;
            
        end
        
        function [dist] = distance(obj, state1, state2)
            %distance computes Dubins path distance between two points.
            
            try
                narginchk(3,3);
                
                % If one of state1 and state2 is single state, skip checking rows for both
                if size(state1, 1) ==  1 || size(state2, 1) ==  1
                    nav.internal.validation.validateStateMatrix(state1, nan, obj.NumStateVariables, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, nan, obj.NumStateVariables, 'distance', 'state2');
                else %otherwise both states have to have the same number of rows
                    nav.internal.validation.validateStateMatrix(state1, nan, obj.NumStateVariables, 'distance', 'state1');
                    nav.internal.validation.validateStateMatrix(state2, size(state1,1), obj.NumStateVariables, 'distance', 'state2');
                end
                
                uavDubPathSeg=connect(obj.DubinsPathSettings, state1, state2);
                dist = cellfun(@(x)x.Length, uavDubPathSeg)';
            catch
                % if dubins connection failed to connect two states, set
                % the distance to inf
                dist = inf;
            end
        end
        
        function interpState = interpolate(obj, state1, state2, fraction)
            %interpolate finds an intermediate state between poses
            
            [state1, state2, fraction] = obj.validateInterpolateInput(state1, state2, fraction);
            
            % Compute Dubin's path and interpolate at different fractions
            uavDubPathSeg=connect(obj.DubinsPathSettings,state1,state2);
            lengths=fraction*uavDubPathSeg{1}.Length;
            [intposes]=interpolate(uavDubPathSeg{1}, lengths);
            
            % Return the [x,y,z,psi] from the interpolated states
            interpState=intposes(:,1:4);
            
        end
        
        function state = sampleUniform(obj, varargin)
            %sampleUniform generates a random state within the sample space.
            
            narginchk(1,4);
            [numSamples, stateBounds] = obj.validateSampleUniformInput(varargin{:});
            
            % Default behavior: Sample uniformly in all state variables
            % based on the user input.a
            obj.UniformDistribution.RandomVariableLimits = stateBounds;
            state = obj.UniformDistribution.sample(numSamples);
            
            state(4)=wrapToPi(state(4));
            r = rand();
            if r < obj.PWGRGuidedSampling
                state = obj.sampleWorkspaceGoalRegion();
            end
            
        end
        
        function state = sampleGaussian(obj, meanState, stdDev, varargin)
            
            narginchk(3,4);
            
            % Default behavior: Sample from a multi-variate normal
            % distribution based on the user input.
            [meanState, stdDev, numSamples] = obj.validateSampleGaussianInput(meanState, stdDev, varargin{:});
            
            % Configure normal distribution for sampling in all state variables
            obj.NormalDistribution.Mean = meanState;
            obj.NormalDistribution.Covariance = diag(stdDev.^2);
            
            % Sample state(s)
            state = obj.NormalDistribution.sample(numSamples);
            
            % Make sure all state samples are within state bounds. This
            % saturation is not ideal, since it distorts the normal
            % distribution on the state boundaries, but similar to what OMPL is doing.
            state = obj.enforceStateBounds(state);
        end
        
        function boundedState = enforceStateBounds(obj, state)
            
            % Default behavior: States are saturated to the [min,max] interval
            nav.internal.validation.validateStateMatrix(state, nan, obj.NumStateVariables, "enforceStateBounds", "state");
            boundedState = state;
            boundedState = min(max(boundedState, obj.StateBounds(:,1)'), ...
                obj.StateBounds(:,2)');
        end
        
        function setWorkspaceGoalRegion(obj,Te_w, bounds)
            %setWorkspaceGoalRegion sets the properties of the target
            %workspace goal region.
            wgr.Targets=Te_w;
            wgr.Bounds = bounds;
            obj.WorkspaceGoalRegion = wgr;
            
        end
        
        function T = sampleWorkspaceGoalRegion(obj)
            %sampleWorkspaceGoalRegion
            %biases sampling of results in region around target.
            
            wgr = obj.WorkspaceGoalRegion;
            dif = wgr.Bounds(:,2) - wgr.Bounds(:,1);
            
            % get random sampled displacement vector in workspace
            dw(1:3,1) = rand(3,1).*dif(1:3,1) + wgr.Bounds(1:3,1);
            dw(4,1)=wrapToPi(rand(1,1)*dif(4,1));
            
            T=dw';
        end
        
        function copyObj = copy(obj)
            %copyObj copies one object to another
            copyObj=ExampleHelperUAVStateSpace;
            copyObj.DubinsPathSettings=copy(obj.DubinsPathSettings);
            copyObj.WorkspaceGoalRegion=obj.WorkspaceGoalRegion;
            copyObj.PWGRGuidedSampling=obj.PWGRGuidedSampling;
            copyObj.AirSpeed=obj.AirSpeed;
            copyObj.StateBounds = obj.StateBounds;
            copyObj.UniformDistribution = obj.UniformDistribution.copy;
            copyObj.NormalDistribution = obj.NormalDistribution.copy;
        end
    end
    
    methods (Static, Access = protected)
        function [mxRoll,arspeed,flightpathangle,bnds] = parseStateSpaceConstuctor(varargin)
            %parseConstructor Parses the inputs to the constructor
            p = matlabshared.autonomous.core.internal.NameValueParser({'MaxRollAngle', 'AirSpeed','FlightPathAngleLimit','Bounds'},...
                {ExampleHelperUAVStateSpace.DefaultMaxRollAngle, ExampleHelperUAVStateSpace.DefaultAirSpeed,ExampleHelperUAVStateSpace.DefaultFlightPathAngleLimit,ExampleHelperUAVStateSpace.DefaultRandomStateBounds});
            if nargin == 0
                p.parse(varargin{:});
            else
                p.parse(varargin{1:end});
            end
            mxRoll = p.parameterValue('MaxRollAngle');
            arspeed = p.parameterValue('AirSpeed');
            flightpathangle=p.parameterValue('FlightPathAngleLimit');
            bnds=p.parameterValue('Bounds');
        end
    end
end
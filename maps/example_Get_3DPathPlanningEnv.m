%this code is used to generat the 3D environment of ICL/NYC maps using the
%Get_3DPathPlanningEnv function 


map_limits= [-300 300;
            -300 300;
            50 150];
map_coordinates= [ 51.4970, 51.5003;
                 -0.1805, -0.1705];
%to get the NYC environment : [40.7672 40.7528);
%                              -73.9859 -73.9601];

OpenStreetMap="ICL_map.osm";%"NYC.osm"

[scene,map3D]=Get_3DPathPlanningEnv(map_coordinates, map_limits,OpenStreetMap);
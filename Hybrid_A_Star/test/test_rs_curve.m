clc
clear
close all

% Algorithm parameters
AlgInfo.start_pos     = [2.5, 2.5, 0.0];
AlgInfo.goal_pos      = [40, 16, pi];
AlgInfo.pos_tolerance = [0.1,  0.1, pi/20];

AlgInfo.costmap_resolution = 0.2;
AlgInfo.collision_resolution = 0.5;
AlgInfo.steer_num = 15;
AlgInfo.theta_num = 90;
AlgInfo.steering_penalty = 1.5;
AlgInfo.steering_change_penalty = 2.0;
AlgInfo.gear_change_penalty = 1.0;

AlgInfo.shot_distance = 10;

% Vehicle parameters
VehInfo.lf = 1; % Distance from C.G. to the front of the car
VehInfo.lr = 1; % Distance from C.G. to the rear of the car
VehInfo.w  = 1; % Car width.
VehInfo.wb = 2.8; % Wheelbase.
VehInfo.min_radius = 5;

% Map information
MapInfo.map_name = "./maps/map_dead_end.png";
MapInfo.resolution = 1;
MapInfo.origin = [0, 0]; 
MapInfo.occupancy_thresh = 1; 

Planner = HybridAStarSearch(AlgInfo, VehInfo, MapInfo);

min_radius = 5;
start_pos = [0.0, 0.0,  0.0];
goal_pos  = [0.5, 0.5, -pi ];

RsPath = reedsSheppConnection;
RsPath.MinTurningRadius = min_radius;
[RsSegment, path_length] = connect(RsPath, start_pos, goal_pos);
poses = interpolate(RsSegment{1}, 0 : 0.2 : path_length);

figure(1)
hold on; grid on; axis equal;
scatter(start_pos(1), start_pos(2), "filled");
plot(poses(:,1), poses(:,2), "LineWidth", 1.5);
% plot(x_list, y_list, "--", "LineWidth", 1.5);

function [AlgInfo, VehInfo, MapInfo] = GetPlannerParams()
    % Algorithm parameters
    AlgInfo.start_pos     = [2.5 , 2.5, 0.0];
    AlgInfo.goal_pos      = [15.5, 15.5, -pi/2];
    AlgInfo.pos_tolerance = [0.1,  0.1, pi/20];
    
    AlgInfo.costmap_resolution = 0.2;
    AlgInfo.collision_resolution = 0.5;
    AlgInfo.steer_num = 15;
    AlgInfo.theta_num = 90;
    
    % Vehicle parameters
    VehInfo.lf = 1; % Distance from C.G. to the front of the car
    VehInfo.lr = 1; % Distance from C.G. to the rear of the car
    VehInfo.w  = 1; % Car width.
    VehInfo.wb = 1; % Wheelbase.
    
    % Map information
    MapInfo.map_name = "./maps/map_dead_end.png";
    MapInfo.resolution = 1;
    MapInfo.origin = [0, 0]; 
    MapInfo.occupancy_thresh = 1; 
end
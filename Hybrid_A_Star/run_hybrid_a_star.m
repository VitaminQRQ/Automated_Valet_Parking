clc
clear
close all

% Algorithm parameters
AlgInfo.start_pos     = [10, 8 ,  pi ];
AlgInfo.goal_pos      = [25, 18,  0.0];
AlgInfo.pos_tolerance = [0.1,  0.1, pi/20];

AlgInfo.costmap_resolution = 0.2;
AlgInfo.collision_resolution = 0.5;
AlgInfo.steer_num = 15;
AlgInfo.theta_num = 90;
AlgInfo.steering_penalty = 2.0;
AlgInfo.steering_change_penalty = 2.0;
AlgInfo.gear_change_penalty = 1.0;

AlgInfo.shot_distance = 10;

% Vehicle parameters
VehInfo.lf = 1.8; % Distance from C.G. to the front of the car
VehInfo.lr = 1.8; % Distance from C.G. to the rear of the car
VehInfo.w  = 2.2; % Car width.
VehInfo.wb = 2.8; % Wheelbase.
VehInfo.min_radius = 5;

% Map information
MapInfo.map_name = "./maps/map_dead_end.png";
MapInfo.resolution = 1;
MapInfo.origin = [0, 0]; 
MapInfo.occupancy_thresh = 1; 

Planner = HybridAStarSearch(AlgInfo, VehInfo, MapInfo);

% Test algorithm
Planner.PlotMap(2);
hold on;
[path, tree] = Planner.PathPlanning();

hold on;
for i = 1 : 10 : size(tree, 1)
    cx = [tree(i, 1), tree(i, 3)];
    cy = [tree(i, 2), tree(i, 4)];
    plot(cx, cy, "color", "#EDB120");
end

if ~isempty(path)
    plot(path(:,1), path(:,2), "LineWidth", 1.5, "Color", "#A2142F");
end
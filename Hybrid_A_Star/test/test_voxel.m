clc
clear
close all

Planner = CreatePlanner();
Planner.PlotMap(1);

start_pos = Planner.start_pos_;
goal_pos = Planner.goal_pos_;

% Global coordinate to grid index / coordinate
[start_row, start_col] = Planner.GlobalToGrid(start_pos(1), ...
                                              start_pos(2), ...
                                              Planner.collision_resolution_);
[goal_row , goal_col ] = Planner.GlobalToGrid(goal_pos(1), ...
                                              goal_pos(2), ...
                                              Planner.collision_resolution_);

% initiallize X and Y
start_X = start_col;
start_Y = start_row;

goal_X = goal_col;
goal_Y = goal_row;

X = start_X;
Y = start_Y;

stepX = sign(goal_X - start_X);
stepY = sign(goal_Y - start_Y);

% Define tMaxX and tMaxY
vec = [goal_col - start_col, goal_row - start_row];

tMaxX = abs(1 / vec(1));
tMaxY = abs(1 / vec(2));

tDeltaX = tMaxX;
tDeltaY = tMaxY;

% Initialize recorder
grid_idx_list = [Y, X];

% Ray tracing
while X ~= goal_col || Y ~= goal_row
    if tMaxX < tMaxY
        tMaxX = tMaxX + tDeltaX;
        X = X + stepX;
    else 
        tMaxY = tMaxY + tDeltaY;
        Y = Y + stepY;
    end
    grid_idx_list = [grid_idx_list; [Y, X]];
end

% Plot grid
for i = 1 : size(grid_idx_list, 1)
    grid_idx = grid_idx_list(i, :);
    row = grid_idx(1);
    col = grid_idx(2);

    [x, y] = Planner.GridToGlobal(row, ...
                                  col, ...
                                  Planner.collision_resolution_);
    Planner.PlotGrid(x, y, Planner.collision_resolution_, "y");
end
plot([Planner.start_pos_(1), Planner.goal_pos_(1)], ...
     [Planner.start_pos_(2), Planner.goal_pos_(2)], ...
     "k", "LineWidth", 1);

function Planner = CreatePlanner()
    % Algorithm parameters
    AlgInfo.start_pos     = [1.25 , 2.25, 0.0];
    AlgInfo.goal_pos      = [10.25, 3.00, 0.0];
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
end
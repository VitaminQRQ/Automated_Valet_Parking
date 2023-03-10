clc
clear
close all

veh_pos = [10, 10, pi/4];

% Plot Map
Planner = CreatePlanner();
Planner.PlotMap(1);

CheckCollision(Planner, veh_pos);

function CheckCollision(Planner, veh_pos)
    veh_w = Planner.w_;
    veh_lf = Planner.lf_;
    veh_lr = Planner.lr_;
    
    corner_rear_left   = [-veh_lr,  veh_w / 2];
    corner_rear_right  = [-veh_lr, -veh_w / 2];
    corner_front_left  = [ veh_lf,  veh_w / 2];
    corner_front_right = [ veh_lf, -veh_w / 2];
    
    boundary_box_x = [corner_rear_left(1)  ; 
                      corner_front_left(1) ; 
                      corner_front_right(1); 
                      corner_rear_right(1) ; 
                      corner_rear_left(1) ];
    
    boundary_box_y = [corner_rear_left(2)  ; 
                      corner_front_left(2) ; 
                      corner_front_right(2); 
                      corner_rear_right(2) ; 
                      corner_rear_left(2) ];
    
    % Rotation matrix
    theta = veh_pos(3);
    rot_matrix = [ cos(theta), sin(theta) ;
                  -sin(theta), cos(theta)];
    
    % Rotation
    boundary = [boundary_box_x, boundary_box_y];
    boundary_rot = boundary * rot_matrix;
     
    % Translation
    x = veh_pos(1);
    y = veh_pos(2);
    boundary_trans = [boundary_rot(:, 1) + x, boundary_rot(:, 2) + y];
    
    % Test ray tracing
    for i = 1 : 4
        boundary_start = boundary_trans(i    , :);
        boundary_goal  = boundary_trans(i + 1, :);
    
        grid_idx_list = RayTracing(Planner, boundary_goal, boundary_start);
    end
    
    % Plot results
    plot(boundary_trans(:,1), boundary_trans(:,2), "LineWidth", 1.5);
    
    axis([min(boundary_trans(:, 1)) - 5, max(boundary_trans(:, 1)) + 5, ...
          min(boundary_trans(:, 2)) - 5, max(boundary_trans(:, 2)) + 5]);
end

function grid_idx_list = RayTracing(Planner, start_pos, goal_pos)
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
    
    for i = 1 : size(grid_idx_list, 1)
        grid_idx = grid_idx_list(i, :);
        row = grid_idx(1);
        col = grid_idx(2);
    
        [x, y] = Planner.GridToGlobal(row, ...
                                      col, ...
                                      Planner.collision_resolution_);
        Planner.PlotGrid(x, y, Planner.collision_resolution_, "y");
    end
end

function Planner = CreatePlanner()
    % Algorithm parameters
    AlgInfo.start_pos     = [2.5, 2.5, 0.0];
    AlgInfo.goal_pos      = [40 , 16 , 0.0];
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
end
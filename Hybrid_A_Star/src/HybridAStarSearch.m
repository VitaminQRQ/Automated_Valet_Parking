classdef HybridAStarSearch < handle
    properties
        % Planning mission
        start_pos_, 
        goal_pos_,
        pos_tolerance_,
    
        % Algorithm params
        primitive_length_,
        primitive_size_,
        state_num_,
        theta_num_,
        theta_resolution_,
        steer_num_,
        path_,
        steering_penalty_,
        steering_change_penalty_,
        gear_change_penalty_,

        % Reeds Shepp curve related params
        RSP_ = reedsSheppConnection;
        shot_distance_,

        % Vehicle Params
        lf_,
        lr_,
        w_,
        wb_,
        min_radius_,

        % Collision checker
        collision_resolution_,

        % Map info
        gridmap_,
        gridmap_resolution_,
        gridmap_width_,
        gridmap_height_,
        
        map_origin_,
        map_lower_x_,
        map_upper_x_,
        map_lower_y_,
        map_upper_y_,

        costmap_,
        costmap_resolution_, 
        costmap_width_,
        costmap_height_

        % TODO: Collision Map
    end

    methods
        function obj = HybridAStarSearch(AlgInfo, VehInfo, MapInfo)
            % Planning Mission
            obj.start_pos_ = AlgInfo.start_pos;
            obj.goal_pos_  = AlgInfo.goal_pos;
            obj.pos_tolerance_ = AlgInfo.pos_tolerance;

            % Algorithm params
            obj.state_num_ = 12;
            
            obj.theta_num_ = AlgInfo.theta_num;
            obj.theta_resolution_ = (2*pi) / obj.theta_num_;
            
            obj.steer_num_ = max(3, AlgInfo.steer_num);
            if mod(obj.steer_num_, 2) == 0
                obj.steer_num_ = obj.steer_num_ + 1;
            end

            obj.collision_resolution_ = AlgInfo.collision_resolution;
            
            obj.steering_penalty_ = AlgInfo.steering_penalty;
            obj.steering_change_penalty_ = AlgInfo.steering_change_penalty;
            obj.gear_change_penalty_ = AlgInfo.gear_change_penalty;
            
            obj.shot_distance_ = AlgInfo.shot_distance;
            obj.min_radius_ = VehInfo.min_radius;
            obj.RSP_.MinTurningRadius = obj.min_radius_;

            % Vehicle params
            obj.lf_ = VehInfo.lf;
            obj.lr_ = VehInfo.lr;
            obj.w_  = VehInfo.w;
            obj.wb_ = VehInfo.wb;
            
            % Read grid map
            map_name = MapInfo.map_name;
            map_pic = imread(map_name);
            map_pic_gray = rgb2gray(map_pic);
            thresh = graythresh(map_pic_gray);
            gridmap = imbinarize(map_pic_gray, thresh);
            gridmap = double(gridmap);
            gridmap(gridmap == 0) = 255;
            gridmap(gridmap == 1) = 0;
            
            % Read grid map info
            origin = MapInfo.origin;
            gridmap_resolution = MapInfo.resolution;
            occupancy_thresh = MapInfo.occupancy_thresh;
            
            gridmap_width = size(gridmap, 2);
            gridmap_height = size(gridmap, 1);

            lower_x = origin(1);
            upper_x = origin(1) + gridmap_resolution * gridmap_width;
            
            lower_y = origin(2);
            upper_y = origin(2) + gridmap_resolution * gridmap_height;
            
            obj.gridmap_ = gridmap;
            obj.gridmap_width_ = gridmap_width;
            obj.gridmap_height_ = gridmap_height;
            obj.gridmap_resolution_ = gridmap_resolution;
            
            obj.map_origin_ = origin;
            obj.map_lower_x_ = lower_x;
            obj.map_upper_x_ = upper_x;
            obj.map_lower_y_ = lower_y;
            obj.map_upper_y_ = upper_y;
            
            % Create cost map
            costmap_resolution = min(AlgInfo.costmap_resolution, gridmap_resolution);
            obj.primitive_length_ = max(1, 1.5*costmap_resolution);
            obj.primitive_size_ = 4;

            costmap_width  = (upper_x - lower_x) / costmap_resolution;
            costmap_height = (upper_y - lower_y) / costmap_resolution;
            costmap = zeros(costmap_height, costmap_width);

            for i = 1 : costmap_height
                for j = 1 : costmap_width
                    [x, y]     = obj.GridToGlobal(i, j, costmap_resolution);
                    [row, col] = obj.GlobalToGrid(x, y, gridmap_resolution);

                    if gridmap(row, col) > occupancy_thresh
                        costmap(i, j) = 1;
                    end
                end
            end
            
            obj.costmap_ = costmap;
            obj.costmap_width_ = costmap_width;
            obj.costmap_height_ = costmap_height;
            obj.costmap_resolution_ = costmap_resolution;
        end % HybridAStarSearch
        
        %% Algorithm
        function [path, search_tree] = PathPlanning(obj)
            fprintf("Start Planning\n");

            % Check validity
            %collision_flag = obj.CheckCollision(obj.start_pos_(1), obj.start_pos_(2));
            collision_flag = obj.CheckCollision_shape(obj.start_pos_(1), ...
                                                      obj.start_pos_(2), ...
                                                      obj.start_pos_(3));
            if collision_flag
                fprintf(2, "Invalid Starting Point!\n");
                path = [];
                search_tree = [];
                obj.PlotMap(999);
                return;
            end

            %collision_flag = obj.CheckCollision(obj.goal_pos_(1), obj.goal_pos_(2));
            collision_flag = obj.CheckCollision_shape(obj.goal_pos_(1), ...
                                                      obj.goal_pos_(2), ...
                                                      obj.goal_pos_(3));
            if collision_flag
                fprintf(2, "Invalid Goal Point!\n");
                path = [];
                search_tree = [];
                obj.PlotMap(999);
                return;
            end

            % Create open_list and close_list
            list_width = obj.state_num_;
            list_height = obj.costmap_height_ * obj.costmap_width_;
            list_depth = obj.theta_num_;

            open_list = ones(list_height, list_width, list_depth) * inf;
            close_list = ones(list_height, list_width, list_depth) * inf;
            
            % Use gpu to accelerate searching
            if canUseGPU()
                fprintf("GPU: on\n");
                open_list = gpuArray(open_list);
            end
            % Define initial state
            start_x = obj.start_pos_(1);
            start_y = obj.start_pos_(2);
            theta = obj.start_pos_(3);

            [start_row, start_col] = obj.GlobalToGrid(start_x, start_y, obj.costmap_resolution_);
            
            start_count = obj.IndexToCount(start_row, start_col, obj.costmap_height_);
            start_stack = obj.ThetaToStack(theta);

            g_cost = 0;
            h_cost = obj.Heuristic(start_x, start_y, theta);
            f_cost = h_cost + g_cost;
            
            initial_delta = 0;    % initial front wheel steering angle
            initial_direction = 0;% initial moving direction

            initial_state = zeros(1, obj.state_num_);
            initial_state(1) = start_x;
            initial_state(2) = start_y;
            initial_state(3) = theta;
            initial_state(4) = start_count;
            initial_state(5) = start_stack;
            initial_state(6) = g_cost;
            initial_state(7) = h_cost;
            initial_state(8) = f_cost;
            initial_state(9) = start_count;
            initial_state(10)= start_stack;
            initial_state(11)= initial_delta;
            initial_state(12)= initial_direction;

            open_list(start_count, :, start_stack) = initial_state;
            
            % Calculate goal pos
            goal_x = obj.goal_pos_(1);
            goal_y = obj.goal_pos_(2);
            goal_theta = obj.goal_pos_(3);

            goal_stack = obj.ThetaToStack(goal_theta);
            
            [goal_row, goal_col] = obj.GlobalToGrid(goal_x, ...
                                                    goal_y, ...
                                                    obj.costmap_resolution_);

            %reach_goal = false;
            open_list_remain = 1;
            
            iter = 1;
            max_iter = 50000;
            search_tree = zeros(max_iter, 4);

            % While loop
            while open_list_remain
                %tic;
                % Pop node with smallest f cost
                open_list_f = open_list(:, 8, :);
                [~, min_f_idx] = min(open_list_f(:));

                % Choose node with minimum f_cost in open_list as current node.
                [current_count, ~, current_stack] = ind2sub(size(open_list_f), min_f_idx);
                current_node = open_list(current_count, :, current_stack);
                %toc;
                
                temp_parent = close_list(current_node(9), :, current_node(10));
                search_path = [current_node(1), current_node(2), temp_parent(1), temp_parent(2)];
                search_tree(iter, :) = search_path;
                
                plot([current_node(1), temp_parent(1)], [current_node(2), temp_parent(2)]);

                % Erase current node from open_list and close it.
                open_list(current_count, :, current_stack) = ones(1, list_width) * inf;
                close_list(current_count, :, current_stack) = current_node;
                open_list_remain = open_list_remain - 1;
                           
                % Retrun path if reach the goal               
                current_x = current_node(1);
                current_y = current_node(2);
                current_theta = current_node(3);

                [current_row, current_col] = obj.GlobalToGrid(current_x, ...
                                                              current_y, ...
                                                              obj.costmap_resolution_);
                % Found a path by grid search                
                if current_row == goal_row ...
                        && current_col == goal_col ...
                        && current_stack == goal_stack
                    disp("Found a path!");
                    fprintf("Total iteration: %d\n", iter);
                    %reach_goal = true;
                    break;
                end
                
                % One shot successfully
                current_pos = [current_x, current_y, current_theta];
                current_dist = norm(obj.goal_pos_(1 : 2) - current_pos(1 : 2));
                rs_flag = false;

                if current_dist < obj.shot_distance_
                    fprintf("One shot\n");
                    [rs_path, ~] = obj.ReedsSheppShot(current_pos, true);
                    for i = 1 : length(rs_path)
                        rs_x = rs_path(i, 1);
                        rs_y = rs_path(i, 2);
                        rs_theta = rs_path(i, 3);

                        % collision_flag = obj.CheckCollision(rs_x, rs_y);
                        collision_flag = obj.CheckCollision_shape(rs_x, rs_y, rs_theta);

                        if collision_flag
                            plot(rs_path(:,1), rs_path(:, 2), "b");
                            fprintf("Oh shit\n");
                            break;
                        end
                    end

                    if collision_flag
                        rs_flag = false;
                    else
                        rs_flag = true;
                        fprintf("One shot successfully!\n");
                        plot(rs_path(:,1), rs_path(:, 2), "g");
                        break;
                    end
                end

                % Expand neighbour nodes
                neighbour_node_list = obj.ExpandNode(current_node);
                
                for i = 1 : size(neighbour_node_list, 1)
                    neighbour_node = neighbour_node_list(i, :);
                    neighbour_f_cost = neighbour_node(8);
                    neighbour_count = neighbour_node(4);
                    neighbour_stack = neighbour_node(5);
                    
                    % neighbour_count == 0 means the node is invalid
                    if neighbour_count == 0
                        continue;
                    end

                    % Add nodes with smaller f cost and have never been
                    % visited before to open list.
                    if neighbour_f_cost < open_list(neighbour_count, 4, neighbour_stack) && ...
                            close_list(neighbour_count, 4, neighbour_stack) == inf
                        
                        if open_list(neighbour_count, 4, neighbour_stack) == inf
                            open_list_remain = open_list_remain + 1;
                        end
                        
                        open_list(neighbour_count, :, neighbour_stack) = neighbour_node;
                    end
                end
                iter = iter + 1;

                if mod(iter, 10) == 0
                    fprintf("Iter %d\n", iter);
                end

                if iter > 50000 || open_list_remain == 0
                    fprintf(2, "Search failed\n");
                    %reach_goal = true;
                    break;
                end
            end
            search_tree = search_tree(1 : iter, :);

            % Retrieve path (new)
            idx = 1;
            reach_start = false;
            
            primitive_size = obj.primitive_size_;
            waypoints_num_approx = obj.costmap_height_ * ...
                                   obj.costmap_width_ * ...
                                   primitive_size;

            path = zeros(waypoints_num_approx, 4);
            
            current_delta = current_node(11);
            current_direction = current_node(12);

            parent_count = current_node(9);
            parent_stack = current_node(10);
            parent_node = close_list(parent_count, :, parent_stack);

            parent_x = parent_node(1);
            parent_y = parent_node(2);
            parent_theta = parent_node(3);

            %parent_delta = parent_node(11);
            parent_direction = parent_node(12);
            
            while ~reach_start
                if parent_count == start_count && parent_stack == start_stack
                    reach_start = true;
                end
                
                [x_list, y_list, theta_list] = obj.VehicleDynamic(parent_x, ...
                                                                  parent_y, ...
                                                                  parent_theta, ...
                                                                  current_delta, ...
                                                                  current_direction);

                direction_list = ones(primitive_size, 1) * current_direction;

                path(idx : idx + primitive_size - 2, :) = [x_list(end : -1 : 2), ...
                                                           y_list(end : -1 : 2), ...
                                                           theta_list(end : -1 : 2), ...
                                                           direction_list(end : -1 : 2)];

                idx = idx + primitive_size - 1;

                current_node = parent_node;
                current_delta = current_node(11);
                current_direction = current_node(12);

                parent_count = current_node(9);
                parent_stack = current_node(10);
                parent_node = close_list(parent_count, :, parent_stack);

                parent_x = parent_node(1);
                parent_y = parent_node(2);
                parent_theta = parent_node(3);
            end
            path(idx, :) = [parent_x, parent_y, parent_theta, parent_direction];
            path = path(idx : -1 : 1, :);

            if rs_flag
                path = [path; rs_path];
            end
            
            fprintf("Done!\n");
        end % PathPlanning
        
        function neighbour_node_list = ExpandNode(obj, current_node)
            % current_node = [1 , x, 
            %                 2 , y, 
            %                 3 , theta, 
            %                 4 , count, 
            %                 5 , theta_stack, 
            %                 6 , g_cost, 
            %                 7 , h_cost, 
            %                 8 , f_cost, 
            %                 9 , parent_count, 
            %                 10, parent_stack,
            %                 11, delta,
            %                 12, direction] 

            neighbour_node_list = zeros(obj.steer_num_ * 2, obj.state_num_);
            x = current_node(1);
            y = current_node(2);
            theta = current_node(3);
            parent_count = current_node(4);
            parent_stack = current_node(5); 
            g_cost = current_node(6);

            prev_delta = current_node(11);
            prev_direction = current_node(12);

            % move backward and forward
            % neighbour_idx = 1;
            max_steer = 35 * (pi / 180);
            % max_steer = pi/15;
            delta_list = linspace(-max_steer, max_steer, obj.steer_num_);
            direction_list = [-1, 1];

            for dir_idx = 1 : 2
                direction = direction_list(dir_idx);
                for i = 1 : obj.steer_num_
                    delta = delta_list(i);
                    
                    [x_list, y_list, theta_list] = obj.VehicleDynamic(x, y, theta, delta, direction);
                    
                    x_next = x_list(end);
                    y_next = y_list(end);
                    theta_next = theta_list(end);
                    theta_next = obj.ModTo2Pi(theta_next);

                    % collision_flag = obj.CheckCollision(x_next, y_next);
                    collision_flag = obj.CheckCollision_shape(x_next, y_next, theta_next);
                    
                    if collision_flag
                        continue;
                    end

                    [row_next, col_next] = obj.GlobalToGrid(x_next, y_next, ...
                                                            obj.costmap_resolution_);

                    count_next = obj.IndexToCount(row_next, col_next, ...
                                                  obj.costmap_height_);
                   
                    stack_next = obj.ThetaToStack(theta_next);

                    % Add penalty to steering / angle change / gear
                    % change
                    moving_cost = obj.primitive_length_;

                    if direction ~= prev_direction && prev_direction ~= 0
                        moving_cost = moving_cost * obj.gear_change_penalty_;
                    end

                    if delta ~= 0
                        moving_cost = moving_cost * obj.steering_penalty_;
                    end
                    
                    if delta ~= prev_delta
                        moving_cost = moving_cost * obj.steering_change_penalty_;
                    end

                    g_cost_next = g_cost + moving_cost;
                    
                    h_cost_next = obj.Heuristic(x_next, y_next, theta_next);
                    f_cost_next = g_cost_next + h_cost_next;

                    neighbour_node    = zeros(1, obj.state_num_);
                    neighbour_node(1) = x_next;
                    neighbour_node(2) = y_next;
                    neighbour_node(3) = theta_next;
                    neighbour_node(4) = count_next;
                    neighbour_node(5) = stack_next;
                    neighbour_node(6) = g_cost_next;
                    neighbour_node(7) = h_cost_next;
                    neighbour_node(8) = f_cost_next;
                    neighbour_node(9) = parent_count;
                    neighbour_node(10)= parent_stack;
                    neighbour_node(11)= delta;
                    neighbour_node(12)= direction;
                    
                    neighbour_idx = i + (dir_idx - 1) * obj.steer_num_;
                    neighbour_node_list(neighbour_idx, :) = neighbour_node;

                    %neighbour_idx = neighbour_idx + 1;
                end
            end
        end % ExpandNode
        
        function [x_list, y_list, theta_list] = VehicleDynamic(obj, x, y, theta, delta, direction)
            D_list = linspace(0, obj.primitive_length_, obj.primitive_size_);
            d = D_list(end) - D_list(end - 1);
            d = d * direction;

            x_list = zeros(obj.primitive_size_, 1);
            y_list = zeros(obj.primitive_size_, 1);
            theta_list = zeros(obj.primitive_size_, 1);
            
            x_list(1) = x;
            y_list(1) = y;
            theta_list(1) = theta;

            for i = 2 : obj.primitive_size_
                x_next = x + d * cos(theta);
                y_next = y + d * sin(theta);
                %theta_next = theta + d * tan(delta) / 1;
                theta_next = theta + d * tan(delta) / obj.wb_;

                x_list(i) = x_next;
                y_list(i) = y_next;
                theta_list(i) = theta_next;

                x = x_next;
                y = y_next;
                theta = theta_next;
            end
        end

        % Calculate heuristic distance between current node [row, col] and
        % goal. 
        function h_cost = Heuristic(obj, current_x, current_y, current_theta)
            tie_breaker = 1.5;
            current_dist = norm([current_x, current_y] - obj.goal_pos_(1 : 2));

            if current_dist > obj.shot_distance_
                h_cost = abs(obj.goal_pos_(1) - current_x) + ...
                         abs(obj.goal_pos_(2) - current_y);
            else
                current_pos = [current_x, current_y, current_theta];
                [~, h_cost] = obj.ReedsSheppShot(current_pos, false);
            end
            h_cost = h_cost * tie_breaker;
        end
        
        % Check if vehicle shape collide with obstacles
        function collision_flag = CheckCollision_shape(obj, x, y, theta)
            collision_flag = 0;
            veh_boundary = obj.GetVehicleShape(x, y, theta);
            
            for i = 1 : 4
                boundary_start = veh_boundary(i + 1, :);
                boundary_goal  = veh_boundary(i    , :);

                collision_flag = obj.RayTracing(boundary_start, boundary_goal);
                
                if collision_flag
                    return;
                end
            end
        end
        
        % Using rotation matrix to get vehicle shape
        function veh_boundary = GetVehicleShape(obj, x, y, theta)
            veh_w = obj.w_;
            veh_lf = obj.lf_;
            veh_lr = obj.lr_;  

            corner_rear_left   = [-veh_lr,  veh_w / 2];
            corner_rear_right  = [-veh_lr, -veh_w / 2];
            corner_front_left  = [ veh_lf,  veh_w / 2];
            corner_front_right = [ veh_lf, -veh_w / 2];
            
            boundary_box_x = [corner_rear_left(1)  ; 
                              corner_front_left(1) ; 
                              corner_front_right(1); 
                              corner_rear_right(1) ; 
                              corner_rear_left(1)  ];
            
            boundary_box_y = [corner_rear_left(2)  ; 
                              corner_front_left(2) ; 
                              corner_front_right(2); 
                              corner_rear_right(2) ; 
                              corner_rear_left(2)  ];

            % Rotation matrix
            rot_matrix = [ cos(theta), sin(theta) ;
                          -sin(theta), cos(theta)];

            % Rotation
            boundary = [boundary_box_x, boundary_box_y];
            boundary_rot = boundary * rot_matrix;

            % Translation
            boundary_trans = [boundary_rot(:, 1) + x, boundary_rot(:, 2) + y];

            % Output
            veh_boundary = boundary_trans;
        end

        % a Fast Voxel Traversal Algorithm
        function collision_flag = RayTracing(obj, ray_start, ray_goal)
            collision_flag = 0;
            
            [start_row, start_col] = obj.GlobalToGrid(ray_start(1), ...
                                                      ray_start(2), ...
                                                      obj.costmap_resolution_);

            [goal_row, goal_col] = obj.GlobalToGrid(ray_goal(1), ...
                                                    ray_goal(2), ...
                                                    obj.costmap_resolution_);
            
            if obj.CheckCollision_idx(start_row, start_col) 
                collision_flag = 1;
                return;
            end
        
            if obj.CheckCollision_idx(goal_row, goal_col)
                collision_flag = 1;
                return;
            end

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
            
            % Ray tracing
            while X ~= goal_col || Y ~= goal_row
                if tMaxX < tMaxY
                    tMaxX = tMaxX + tDeltaX;
                    X = X + stepX;
                else 
                    tMaxY = tMaxY + tDeltaY;
                    Y = Y + stepY;
                end
                
                current_col = X;
                current_row = Y;

                if obj.CheckCollision_idx(current_row, current_col)
                    collision_flag = 1;
                    return;
                end
            end
        end

        % Check if current node [x, y] collide with obstacles. (Using
        % global coordinate.
        function collision_flag = CheckCollision(obj, x, y)
            [row, col] = obj.GlobalToGrid(x, y, obj.costmap_resolution_);
            collision_flag = obj.CheckCollision_idx(row, col);
        end

        % Check if current node [row, col] collide with obstacles. (Using
        % grid map index)
        function collision_flag = CheckCollision_idx(obj, row, col)
            collision_flag = false;

            if row > obj.costmap_height_ || row < 1
                collision_flag = true;
                return;
            end
            
            if col > obj.costmap_width_ || col < 1
                collision_flag = true;
                return;
            end

            if obj.costmap_(row, col) > 0
                collision_flag = true;
                return;
            end
        end % CheckCollision_idx
        
        function [rs_path, path_length] = ReedsSheppShot(obj, current_pos, interp_flag)
            start_pos = gather(current_pos);
            goal_pos = obj.goal_pos_;
            [path_segment, path_length] = connect(obj.RSP_, start_pos, goal_pos);
            
            if interp_flag
                [rs_path, rs_dir] = interpolate(path_segment{1}, 0 : obj.costmap_resolution_ : path_length);
                rs_path = [rs_path, rs_dir];
            else
                rs_path = [];
            end
        end

        %% Utils
        % Plot grid map
        function PlotMap(obj, fignum)
            % Create figure
            figure(fignum);
            hold on; grid on; axis equal;
            
            % Plot map boundaries
            map_edges = ...
                [obj.map_lower_x_, obj.map_lower_y_;
                 obj.map_upper_x_, obj.map_lower_y_;
                 obj.map_upper_x_, obj.map_upper_y_;
                 obj.map_lower_x_, obj.map_upper_y_;
                 obj.map_lower_x_, obj.map_lower_y_;];
            fill(map_edges(:,1), map_edges(:,2), [0.9, 0.9, 0.9]);

            % Plot obstacles
            gridmap_height = size(obj.gridmap_, 1);
            gridmap_width  = size(obj.gridmap_, 2);

            for i = 1 : gridmap_height
               for j = 1 : gridmap_width
                   [x, y] = obj.GridToGlobal(i, j, obj.gridmap_resolution_);
       
                   if obj.gridmap_(i, j) > 0
                       obj.PlotGrid(x, y, obj.gridmap_resolution_, [0.1, 0.1, 0.1]);
                   end
               end
            end
        
            % Plot starting point
            %grid_length = obj.costmap_resolution_;
            x = obj.start_pos_(1);
            y = obj.start_pos_(2);
            theta = obj.start_pos_(3);
            veh_boundary = obj.GetVehicleShape(x, y, theta);
            plot(veh_boundary(:, 1), veh_boundary(:, 2), "LineWidth", 1);
            %obj.PlotGrid(x, y, obj.collision_resolution_, 'b');

            % Plot starting point and goal point
            x = obj.goal_pos_(1);
            y = obj.goal_pos_(2);
            theta = obj.goal_pos_(3);
            veh_boundary = obj.GetVehicleShape(x, y, theta);
            plot(veh_boundary(:, 1), veh_boundary(:, 2), "LineWidth", 1);
            %obj.PlotGrid(x, y, obj.collision_resolution_, 'r');
        end % PlotMap
        
        % Plot grid
        function PlotGrid(~, x, y, resolution, grid_color)
            grid_length = resolution;
            grid_edges = [x - grid_length / 2, y - grid_length / 2;
                          x + grid_length / 2, y - grid_length / 2;
                          x + grid_length / 2, y + grid_length / 2;
                          x - grid_length / 2, y + grid_length / 2];  
            fill(grid_edges(:,1)', grid_edges(:, 2)', grid_color);
        end
        
        % Transfer grid map index to coordinate
        function [x, y] = GridToGlobal(obj, row, col, resolution)
            x = obj.map_lower_x_ + (col - 0.5) * resolution;
            y = obj.map_upper_y_ - (row - 0.5) * resolution;
        end % GridToGlobal
        
        % Transfer coordinate to grid map index
        function [row, col] = GlobalToGrid(obj, x, y, resolution)
            row = round((obj.map_upper_y_ - y) / resolution + 0.5); 
            row = max(row, 1);

            col = round((x - obj.map_lower_x_) / resolution + 0.5);
            col = max(col, 1);
        end % GlobalToGrid
        
        % Transfer grid map index [row, col] to linear index
        function count = IndexToCount(~, row, col, height)
            count = (col - 1) * height + row;
        end
        
        % Transfer linear index to grid map index [row, col]
        function [row, col] = CountToIndex(~, count, height)
            row = rem(count - 1, height) + 1;
            col = (count - row) / height + 1;
        end
        
        % Takes an angle in radians and returns which "stack" in the 3D
        % configuration space this angle corresponds to.
        function stack = ThetaToStack(obj, theta)
            theta = obj.ModTo2Pi(theta);
            stack = round(theta / obj.theta_resolution_) + 1;
            
            if stack >= obj.theta_num_ + 1
                theta = 2 * pi - theta;
                stack = round(theta / obj.theta_resolution_) + 1;
            end
        end

        % Takes in a stack in the 3D configuration space and returns an 
        % angle in radians.
        function theta = StackToTheta(obj, stack)
            theta = (stack - 1) * obj.theta_resolution_;
        end

        % Wrap angle in radians to [0, 2*pi]
        function theta = ModTo2Pi(~, angle)
            theta = mod(angle + 2 * pi, 2 * pi);
        end
    end
end
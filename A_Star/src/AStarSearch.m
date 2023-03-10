classdef AStarSearch < handle
    properties
        % Algorithm info
        start_pos_,
        goal_pos_,
        state_num_,
        path,

        % Map info
        gridmap_,
        gridmap_resolution_,
        gridmap_width_,
        gridmap_height_,
        %grid_occupancy_thresh_,
        %grid_free_thresh_,
        
        map_origin_,
        map_lower_x_,
        map_upper_x_,
        map_lower_y_,
        map_upper_y_,

        costmap_,
        costmap_resolution_, 
        costmap_width_,
        costmap_height_
    end

    methods
        function obj = AStarSearch(start_pos, goal_pos, costmap_resolution, MapInfo)
            % Planning Mission
            obj.start_pos_ = start_pos;
            obj.goal_pos_  = goal_pos;
            obj.state_num_ = 7;

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
            costmap_resolution = min(costmap_resolution, gridmap_resolution);
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
        end % AStarSearch
        
        %% Algorithm
        function path = PathPlanning(obj)
            [start_row, start_col] = obj.GlobalToGrid(obj.start_pos_(1), ...
                                                      obj.start_pos_(2), ...
                                                      obj.costmap_resolution_);
            start_pos = [start_row, start_col];
            
            [goal_row, goal_col] = obj.GlobalToGrid(obj.goal_pos_(1), ...
                                                    obj.goal_pos_(2), ...
                                                    obj.costmap_resolution_);
            goal_pos  = [goal_row, goal_col];
            path = [];
            
            % Check validity
            if obj.CheckCollision_idx(start_pos(1), start_pos(2))
                fprintf(2, "Invalid Starting Point!\n");
                path = [];
                obj.PlotMap(1);
                return;
            end
            
            if obj.CheckCollision_idx(goal_pos(1), goal_pos(2))
                fprintf(2, "Invalid Goal Point\n");
                path = [];
                obj.PlotMap(1);
                return;
            end

            % state = [x, y, g, h, f, parent_count]
            state_num = obj.state_num_;
            costmap_size = obj.costmap_height_ * obj.costmap_width_;
            open_list  = ones(costmap_size, state_num) * inf;
            close_list = ones(costmap_size, state_num) * inf;
            
            start_count = obj.IndexToCount(start_pos(1), ...
                                           start_pos(2), ...
                                           obj.costmap_height_);
            
            % Define initial state
            initial_state = zeros(1, state_num);
            initial_state(1) = start_pos(1); % row
            initial_state(2) = start_pos(2); % col
            initial_state(3) = start_count;  % lienar index of [row, col]
            initial_state(4) = 0;            % g_cost
            initial_state(5) = 0;            % h_cost
            initial_state(6) = 0;            % f_cost = g_cost + h_cost
            initial_state(7) = start_count;  % parent node linear index
            
            open_list(start_count, :) = initial_state;
            close_list(start_count, :) = initial_state;

            reach_goal = false;
            open_list_remain = 1;

            while open_list_remain > 0 && ~reach_goal
                % Choose node with minimum f_cost in open_list as current node.
                [~, current_node_idx] = min(open_list(:, 6));
                current_node = open_list(current_node_idx, :);
                
                if current_node(1) == goal_row && current_node(2) == goal_col
                    reach_goal = true;
                    break;
                end

                % Remove current node from open_list
                open_list(current_node_idx, :) = ones(1, state_num) * inf; 
                open_list_remain = open_list_remain - 1;

                % Add current node to close_list
                close_list(current_node_idx, :) = current_node;

                % Expand current node.
                neighbour_node_list = obj.ExpandNode(current_node); 

                % Update open_list and close_list
                for i = 1 : size(neighbour_node_list, 1)
                    neighbour = neighbour_node_list(i, :);
                    neighbour_count = neighbour(3);
                    neighbour_f_cost = neighbour(6);
                    
                    if neighbour_count == 0
                        continue;
                    end

                    if neighbour_f_cost < open_list(neighbour_count, 6) ...
                               && close_list(neighbour_count, 6) == inf
                        open_list(neighbour_count, :) = neighbour;
                        open_list_remain = open_list_remain + 1;
                    end
                end
            end
            
            % Retrieve path
            if reach_goal
                idx = 1;
                path = zeros(costmap_size, 2);
                reach_start = false;
                
                while ~reach_start
                    [current_x, current_y] = obj.GridToGlobal(current_node(1), ...
                                                              current_node(2), ...
                                                              obj.costmap_resolution_);
                    path(idx, :) = [current_x, current_y];
                    
                    parent_idx = current_node(7);
                    parent_node = close_list(parent_idx, :);

                    if parent_node(1) == start_row && parent_node(2) == start_col
                        reach_start = true;
                    end

                    current_node = parent_node;
                    idx = idx + 1;
                end
                
                path(idx, :) = obj.start_pos_;
                path = path(idx : -1 : 1, :);

                obj.PlotMap(2);
                plot(path(:,1), path(:,2), "LineWidth", 1.5);
            end
 
            obj.path = path;
        end % PathPlanning
        
        function neighbour_node_list = ExpandNode(obj, current_node)
            current_row = current_node(1);
            current_col = current_node(2);
            current_count = current_node(3);
            current_g_cost = current_node(4);
            
            neighbour_rows = [current_row - 1, current_row, current_row + 1];
            neighbour_cols = [current_col - 1, current_col, current_col + 1];

            neighbour_costs = [1.414, 1.000, 1.414;
                               1.000, 0.000, 1.000;
                               1.414, 1.000, 1.414]; 
            
            neighbour_node_list = zeros(8, obj.state_num_);
            
            idx = 1;
            for i = 1 : 3
                row = neighbour_rows(i);
                for j = 1 : 3
                    col = neighbour_cols(j);
                    cost = neighbour_costs(i, j);
                    
                    % Skip current point
                    if cost == 0 
                        continue;
                    end
                    
                    % Skip obstacles
                    if obj.CheckCollision_idx(row, col)
                        continue;
                    end
                    
                    neighbour_count = obj.IndexToCount(row, col, obj.costmap_height_);
                    h_cost = obj.Heuristic(row, col);
                    g_cost = current_g_cost + cost;
    
                    neighbour_node_list(idx, 1) = row;
                    neighbour_node_list(idx, 2) = col;
                    neighbour_node_list(idx, 3) = neighbour_count;
                    neighbour_node_list(idx, 4) = g_cost;
                    neighbour_node_list(idx, 5) = h_cost;
                    neighbour_node_list(idx, 6) = g_cost + h_cost;
                    neighbour_node_list(idx, 7) = current_count;

                    idx = idx + 1;
                end
            end
        end % ExpandNode

        function h_cost = Heuristic(obj, row, col)
            [target_row, target_col] = obj.GlobalToGrid(obj.goal_pos_(1), ...
                                                        obj.goal_pos_(2), ...
                                                        obj.costmap_resolution_);
            %h_cost = sqrt((target_row - row)^2 + (target_col - col)^2);
            h_cost = abs(target_row - row) + abs(target_col - col);
        end

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

        function collision_flag = CheckCollision(obj, x, y)
            collision_flag = false;
            [row, col] = obj.GlobalToGrid(x, y, obj.costmap_resolution_);

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
        end % CheckCollision
        

        %% Utils
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
            scatter(x, y, 30, "filled");
            
            % Plot starting point and goal point
            x = obj.goal_pos_(1);
            y = obj.goal_pos_(2);         
            scatter(x, y, 30, "filled");
        end % PlotMap
        
        function PlotGrid(~, x, y, resolution, grid_color)
            grid_length = resolution;
            grid_edges = [x - grid_length / 2, y - grid_length / 2;
                          x + grid_length / 2, y - grid_length / 2;
                          x + grid_length / 2, y + grid_length / 2;
                          x - grid_length / 2, y + grid_length / 2];  
            fill(grid_edges(:,1)', grid_edges(:, 2)', grid_color);
        end

        function [x, y] = GridToGlobal(obj, row, col, resolution)
            x = obj.map_lower_x_ + (col - 0.5) * resolution;
            y = obj.map_upper_y_ - (row - 0.5) * resolution;
        end % GridToGlobal

        function [row, col] = GlobalToGrid(obj, x, y, resolution)
            row = round((obj.map_upper_y_ - y) / resolution + 0.5); 
            col = round((x - obj.map_lower_x_) / resolution + 0.5);
        end % GlobalToGrid

        function count = IndexToCount(~, row, col, height)
            count = (col - 1) * height + row;
        end

        function [row, col] = CountToIndex(~, count, height)
            row = rem(count - 1, height) + 1;
            col = (count - row) / height + 1;
        end
    end
end
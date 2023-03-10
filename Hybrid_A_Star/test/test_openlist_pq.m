clc
clear
close all

% Define basic node
empty_node = struct();
empty_node.f_cost = Inf;
empty_node.other  = Inf;

% Define start node
start_node = empty_node;
start_node.f_cost = 99;
start_node.other = "Hello";

% Create priority queue
open_list_pq = PriorityQueue(start_node, empty_node);

% Expand neighbours
neighbour_list = repmat(empty_node, [40, 1]);
for i = 1 : 40
    neighbour_list(i, :).f_cost = randi([1, 99]);
    neighbour_list(i, :).other = "Hello";
end

% Add neighbours to openList
for i = 1 : 40
    open_list_pq.PushNode(neighbour_list(i));
end

% Debug
open_list_pq.PrintHeap();
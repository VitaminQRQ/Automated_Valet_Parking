clc
clear
close all

empty_node = struct();
empty_node.f_cost = Inf;
empty_node.other  = Inf;

node_list = repmat(empty_node, [10, 1]);
for i = 1 : length(node_list)
    node_list(i).f_cost = randi([10, 99]);
    node_list(i).other = "Hello";
end

fprintf("Test BuildMinimumHeap(): \n\n");
OpenList_PQ = PriorityQueue(node_list, empty_node); 
OpenList_PQ.PrintHeap();

fprintf("Test PopNode(): \n\n");
min_f_node = OpenList_PQ.PopNode();
OpenList_PQ.PrintHeap();

fprintf("Test PushNode(): \n\n");
OpenList_PQ_new = PriorityQueue(node_list(1), empty_node);

for i = 2 : length(node_list)
    empty_node = node_list(i);
    OpenList_PQ_new.PushNode(empty_node);
end

OpenList_PQ_new.PrintHeap();

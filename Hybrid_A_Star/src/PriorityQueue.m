classdef PriorityQueue < handle
    % A priority queue implemented by minimum heap.
    % The priority is sorted using `node.f_cost`.
    
    properties
        data_size_, % Number of element in the heap
        data_,      % Using list to store heap
        empty_node_,
    end
    
    methods
        %% 初始化
        function obj = PriorityQueue(node_list, empty_node)
            obj.data_size_ = length(node_list);
            obj.empty_node_ = empty_node;

            heap_height = floor(log2(obj.data_size_)) + 1;
            heap_size = 2 ^ heap_height - 1;

            obj.data_ = repmat(empty_node, [heap_size, 1]);
            obj.data_(1 : obj.data_size_) = node_list;
            obj.BuildMinimumHeap();
        end
        
        %% 将数组转换为最小堆
        function BuildMinimumHeap(obj)
            for i = obj.data_size_ / 2 : -1 : 1
                obj.Heapify(i);
            end
        end
        
        %% 通过下沉操作使得 idx 所描述的节点满足堆序性
        function Heapify(obj, idx)
            % 获取左右子节点
            left_child = obj.GetLeftChild(idx);
            right_child = obj.GetRightChild(idx);
            
            % 假设当前节点满足堆序性
            smallest_node = idx;
            smallest_value = obj.data_(idx).f_cost;

            left_valid_flag = true;
            right_valid_flag = true;
            
            % 判断左子节点是否存在
            if left_child > obj.data_size_
                left_valid_flag = false;
                left_value = Inf;
            else
                left_value = obj.data_(left_child).f_cost;
            end

            % 判断右子节点是否存在
            if right_child > obj.data_size_
                right_valid_flag = false;
                right_value = Inf;
            else
                right_value = obj.data_(right_child).f_cost;
            end
            
            % 存在左孩子，且左孩子的 f_cost 小于父节点
            if left_valid_flag && left_value < smallest_value
                smallest_node = left_child;
                smallest_value = obj.data_(smallest_node).f_cost;
            end
            
            % 存在右孩子，且右孩子的 f_cost 小于父节点
            if right_valid_flag && right_value < smallest_value
                smallest_node = right_child;
            end

            % smallest_node 没发生变化说明满足堆序性，无需后续操作
            if smallest_node == idx
                return;
            end

            obj.Swap(idx, smallest_node);
            obj.Heapify(smallest_node);
        end
        
        % 获取 idx 所描述节点的左孩子
        function left_child = GetLeftChild(~, idx)
            left_child = idx * 2;
        end

        % 获取 idx 所描述节点的右孩子
        function right_child = GetRightChild(~, idx)
            right_child = idx * 2 + 1;
        end
        
        % 交换堆中两个节点的位置
        function Swap(obj, idx_1, idx_2)
            temp_node = obj.data_(idx_1);
            obj.data_(idx_1) = obj.data_(idx_2);
            obj.data_(idx_2) = temp_node;
        end
        
        % 弹出堆顶的元素，并重新排序
        function node = PopNode(obj)
            node = obj.data_(1);
            obj.data_(1) = obj.data_(obj.data_size_);
            obj.data_(obj.data_size_) = obj.empty_node_;
            obj.data_size_ = obj.data_size_ - 1;
            obj.Heapify(1);
        end

        % 向堆中添加元素
        function PushNode(obj, node)
            % 如果存储空间不够，则拓宽一倍
            obj.data_size_ = obj.data_size_ + 1;

            if obj.data_size_ > length(obj.data_)
                obj.data_ = [obj.data_; repmat(obj.empty_node_, [obj.data_size_, 1])];
            end

            obj.data_(obj.data_size_) = node;

            % 不断上浮，直到找到属于自己的位置
            idx = obj.data_size_;
            while idx >= 2
                parent = obj.GetParent(idx);
                
                current_value = obj.data_(idx).f_cost;
                parent_value = obj.data_(parent).f_cost;
                if current_value < parent_value
                    obj.Swap(idx, parent);
                end

                idx = parent;
            end
        end

        function parent = GetParent(~, idx)
            parent = floor(idx / 2);
        end

        %% Animation
        function PrintHeap(obj)
            return_power = 0;
            return_counter = 0;
            
            gap = 3;
            tree_width = 2 ^ floor(log2(obj.data_size_));
            tree_space_width = gap * tree_width + (tree_width - 1) * gap;
            space_num = tree_space_width / 2 - 1;

            for i = 1 : length(obj.data_)
                for j = 1 : space_num
                    fprintf(" ");
                end
                fprintf(" %2d   ", obj.data_(i).f_cost);
                return_counter = return_counter + 1;
                for j = 1 : space_num
                    fprintf(" ");
                end

                if 2^return_power == return_counter
                    fprintf("\n");
                    return_power = return_power + 1;
                    return_counter = 0;
                    space_num = space_num / 2 - 1;
                end
            end
            fprintf("\n");
        end
    end
end


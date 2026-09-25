#pragma once

#include "grid_map.h"
#include <vector>

struct Node
{
    // 构造函数，只需要知道坐标
    Node(int row, int col);

    // 节点坐标
    int row_, col_;
    // 从起点到节点的实际代价
    int cost_so_far_;
    // 直线估计代价
    int estimated_cost_to_goal_;
    // 预估的最终代价，为前两者的和
    int estimated_total_cost() const;
};

int manhattan_distance(const Node& a, const Node& b);
std::vector<std::pair<int, int>> astar_search(const GridMap& map, Node start_node, Node goal_node);
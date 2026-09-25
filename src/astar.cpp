#include "astar.h"
#include <cstdlib>
#include <limits>
#include <queue>
#include <iostream>
#include <utility>
#include <vector>

Node::Node(int row, int col) : 
    row_(row), col_(col)
    // 初始时当前代价设为无穷大，表示尚未到达（0 保留为起点）
    , cost_so_far_(std::numeric_limits<int>::max())
    , estimated_cost_to_goal_(0)
    {};

int Node::estimated_total_cost() const
    { return cost_so_far_ + estimated_cost_to_goal_; }

// 计算节点间的曼哈顿距离
int manhattan_distance(const Node& a, const Node& b)
{
    return std::abs((a.row_ - b.row_)) + std::abs((a.col_ - b.col_));
}

std::vector<std::pair<int, int>> astar_search(const GridMap& map, Node start_node, Node goal_node)
{
    start_node.cost_so_far_ = 0;
    // 起点的代价为到终点的曼哈顿距离
    start_node.estimated_cost_to_goal_ = manhattan_distance(start_node, goal_node);

    // 自定义比较器，总 cost 高的排在队列后 
    struct CompareNodeCost
    {
        bool operator()(const Node& a, const Node& b) const 
            { return a.estimated_total_cost() > b.estimated_total_cost(); }
    };

    // 邻居方向数组，用于后续遍历
    const std::pair<int, int> directions[4] = 
    {
        {-1,  0},   // 上
        { 1,  0},   // 下
        { 0, -1},   // 左
        { 0,  1}    // 右
    };

    // open_list 记录待处理节点
    std::priority_queue<Node, std::vector<Node>, CompareNodeCost> open_list;
    open_list.push(start_node);

    // closed_list 记录已被拓展（考虑过邻居）的节点
    std::vector<std::vector<bool>> closed_list(map.get_row_count(), std::vector<bool>(map.get_col_count(), false));

    // parent 记录遍历过程中被考虑过为优选的节点的上一个节点
    std::vector<std::vector<std::pair<int, int>>> parent(map.get_row_count(), std::vector<std::pair<int, int>>(map.get_col_count(), {0, 0}));

    // 记录是否找到终点
    bool flag = false;

    while (!open_list.empty())
    {
        // 关注推测代价最小的节点
        Node current_node = open_list.top();
        // 将其移除
        open_list.pop();

        // 若已是终点，则搜索成功
        if (current_node.row_ == goal_node.row_ && current_node.col_ == goal_node.col_) { flag = true; std::cout << "找到路径" << std::endl; break; }
        // 若节点已被处理，则跳过
        if (closed_list[current_node.row_][current_node.col_]) continue;
        closed_list[current_node.row_][current_node.col_] = true;

        // 遍历邻居节点
        for (const auto& [drow, dcol] : directions)
        {
            // 计算邻居节点的坐标
            int nearby_row = current_node.row_ + drow;
            int nearby_col = current_node.col_ + dcol;
            // 若邻居节点满足：在地图内、不为墙体、未被遍历
            if (map.inBound(nearby_row, nearby_col) &&
                !map.isObstacle(nearby_row, nearby_col) &&
                !closed_list[nearby_row][nearby_col])
            {
                // 满足条件再初始化
                Node neighbor_node(nearby_row, nearby_col);
                neighbor_node.cost_so_far_ = current_node.cost_so_far_ + 1;
                neighbor_node.estimated_cost_to_goal_ = manhattan_distance(neighbor_node, goal_node);

                // 记录当前节点作为邻居节点的上一个节点
                parent[nearby_row][nearby_col] = {current_node.row_, current_node.col_};
                open_list.push(neighbor_node);
            }
        }
    }
    if (!flag) { std::cout << "寻路失败" << std::endl; }
    return {};
}
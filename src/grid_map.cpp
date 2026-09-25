#include "grid_map.h"

#include <vector>
#include <sstream>
#include <iostream>

GridMap::GridMap(int row_count, int col_count, int initial_value)
{
    row_count_ = row_count;
    col_count_ = col_count;
    map_ = std::vector<std::vector<int>>(row_count, std::vector<int>(col_count, initial_value));
}

GridMap::GridMap(const std::vector<std::vector<int>> & map)
{
    map_ = map;
    row_count_ = map.size();
    col_count_ = map[0].size();
}

bool GridMap::inBound(int row, int col) const
{
    return row >= 0 && row < row_count_
        && col >= 0 && col < col_count_;
}

bool GridMap::isObstacle(int row, int col) const 
{
    if (!inBound(row, col)) return true;   // 地图外为墙
    return map_[row][col];
}

void GridMap::setPoint(int row, int col, int value) 
{
    if (!inBound(row, col)) return;        // 越界时直接忽略
    map_[row][col] = value;
}

void GridMap::print_map() const
{
    std::ostringstream stream;

    for (const auto& row : map_)
    {
        for (const auto& value : row)
        {
            if (&value == &row.back()) { stream << value; }      // 每行结尾不加空格
            else { stream << value << ' '; }                     // 否则要加空格
        }
        stream << '\n';
    }

    std::cout << stream.str();
}
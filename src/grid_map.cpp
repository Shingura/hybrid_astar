#include "grid_map.h"

#include <stdexcept>
#include <fstream>
#include <vector>
#include <sstream>
#include <iostream>

GridMap::GridMap(int row_count, int col_count, int initial_value)
    : row_count_(row_count)
    , col_count_(col_count)
    , map_(row_count, std::vector<int>(col_count, initial_value)) {}

GridMap::GridMap(const std::vector<std::vector<int>> & map)
    : map_(map)
{
    if (map_.empty() || map_[0].empty()) { throw std::invalid_argument("地图数据为空"); }
    row_count_ = map_.size();
    col_count_ = map_[0].size();
}

bool GridMap::inBound(int row, int col) const
{
    return row >= 0 && row < row_count_
        && col >= 0 && col < col_count_;
}

bool GridMap::isObstacle(int row, int col) const 
{
    if (!inBound(row, col)) return true;   // 地图外为墙
    return map_[row][col] != 0;
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

GridMap load_map(const std::string & path_to_map)
{
    std::vector<std::vector<int>> map_vector;

    // 读取文件
    std::ifstream file(path_to_map);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    // 当前行
    std::string line;

    // 从文件中遍历行
    while (std::getline(file, line))
    {
        // 行向量，遍历完后推入二维数组
        std::vector<int> line_vector;
        for (auto& element : line)
        {
            if (element == '#') { line_vector.push_back(1); }
            if (element == '.') { line_vector.push_back(0); }
        }

        // 将当前行推入二维数组
        map_vector.push_back(line_vector);
    }
    GridMap map(map_vector);
    return map;
}
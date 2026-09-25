#pragma once
#include <vector>
#include <string>

class GridMap
{
    public:
        GridMap(int row_count_, int col_count_, int initial_value_ = 0);    // 创建地图时初始值默认为 0
        GridMap(const std::vector<std::vector<int>> & map);                       // 从二维数组直接创建地图

        int get_row_count() const { return row_count_; } 
        int get_col_count() const { return col_count_; }

        // 检查坐标是否合法
        bool inBound(int row, int col) const;
        // 检查是否为障碍点
        bool isObstacle(int row, int col) const;
        // 设定坐标
        void setPoint(int row, int col, int value);

        // 打印地图
        void print_map() const;

    private:
        // 行数、列数
        int row_count_, col_count_;

        // 地图本身
        std::vector<std::vector<int>> map_;
};

GridMap load_map(const std::string & path_to_map);
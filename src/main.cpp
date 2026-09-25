#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "grid_map.h"

int main()
{
    std::vector<std::vector<int>> map_vector;

    // 读取文件
    std::ifstream file("maps/map1.txt");
    if (!file.is_open()) { std::cerr << "无法打开文件" << std::endl; return 1; }

    // 当前行
    std::string line;
    int line_num = 0;

    // 从文件中遍历行
    while (std::getline(file, line))
    {
        line_num++;

        std::vector<int> line_vector;
        for (auto& element : line)
        {
            if (element == '#') { line_vector.push_back(1); }
            if (element == '.') { line_vector.push_back(0); }
        }

        // 将当前行推入二维数组
        map_vector.push_back(line_vector);
    }

    // 用二维数组直接创建地图
    GridMap map(map_vector);

    map.print_map();
    return 0;
}
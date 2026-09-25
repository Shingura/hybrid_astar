#include <iostream>
#include <string>

#include "astar.h"
#include "grid_map.h"

int main()
{
    GridMap map = load_map("maps/map1.txt");
    std::cout << "原始地图如下：\n";
    map.print_map();
    std::cout << std::endl;

    Node start(1, 1), goal(10, 18);
    auto path = astar_search(map, start, goal);
    for (auto [row, col] : path) { map.setPoint(row, col, 5); };
    std::cout << "寻路路径如下：\n";
    map.print_map();
    return 0;
}
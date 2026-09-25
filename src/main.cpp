#include <iostream>
#include <string>

#include "astar.h"
#include "grid_map.h"

int main()
{
    GridMap map = load_map("maps/map1.txt");
    map.print_map();
    std::cout << std::endl;

    Node start(1, 1), goal(10, 18);
    astar_search(map, start, goal);
    return 0;
}
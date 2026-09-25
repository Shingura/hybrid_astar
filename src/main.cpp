#include <filesystem>
#include <iostream>
#include <string>

#include "astar.h"
#include "grid_map.h"
#include "hybrid_astar.h"
#include "image.h"

int main()
{
    std::filesystem::create_directories("output");
    GridMap map = load_map("maps/map1.txt");
    constexpr int SCALE = 24;   // 每格 24 像素

    // ---------- 1. 栅格 A* ----------
    Node start(1, 1), goal(10, 18);
    auto grid_path = astar_search(map, start, goal);
    std::cout << "[栅格 A*] 路径格数: " << grid_path.size() << '\n';

    Image img1 = render_map(map, SCALE);
    draw_grid_path(img1, grid_path, SCALE, {80, 140, 255});
    img1.save("output/grid_astar.ppm");

    // ---------- 2. Hybrid A* ----------
    // 格子 (row, col) 的中心在连续坐标里是 (col + 0.5, row + 0.5)，theta = 0 表示车头朝右
    VehicleState vstart{1.5, 1.5, 0.0};
    VehicleState vgoal{18.5, 10.5, 0.0};
    auto res = hybrid_astar_search(map, vstart, vgoal);

    if (res.success)
    {
        std::cout << "[Hybrid A*] 成功，路径点数: " << res.path.size()
                  << "，弧长: " << res.path_length
                  << "，扩展节点: " << res.expanded_nodes << '\n';

        std::vector<std::pair<double, double>> points;
        for (const auto& s : res.path) points.emplace_back(s.x, s.y);

        Image img2 = render_map(map, SCALE);
        draw_polyline(img2, points, SCALE, {230, 60, 60});
        img2.save("output/hybrid_astar.ppm");
        std::cout << "图片已保存到 output/ 目录（PPM 格式，可直接用图片查看器打开）\n";
    }
    else
    {
        std::cout << "[Hybrid A*] 寻路失败\n";
    }
    return 0;
}

// 极简测试：不用框架，CHECK 失败即打印并计数，最后按计数返回退出码
#include <cmath>
#include <iostream>
#include <vector>

#include "astar.h"
#include "grid_map.h"
#include "hybrid_astar.h"

static int failures = 0;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "FAIL " << __FILE__ << ':' << __LINE__              \
                      << "  " #cond << '\n';                                 \
            ++failures;                                                      \
        }                                                                    \
    } while (0)

static double angle_diff(double a, double b)
{
    double d = std::fmod(a - b, 2.0 * M_PI);
    if (d > M_PI) d -= 2.0 * M_PI;
    if (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

int main()
{
    GridMap map = load_map("maps/map1.txt");

    // ========== 栅格 A* ==========
    {
        Node start(1, 1), goal(10, 18);
        auto path = astar_search(map, start, goal);

        CHECK(!path.empty());                                          // 路径存在
        CHECK(path.front() == std::make_pair(1, 1));                   // 起点正确
        CHECK(path.back()  == std::make_pair(10, 18));                 // 终点正确
        CHECK(path.size() == 27);   // 最优：曼哈顿距离 26 步 = 27 格

        for (size_t i = 0; i < path.size(); ++i)
        {
            auto [r, c] = path[i];
            CHECK(!map.isObstacle(r, c));                              // 不穿墙
            if (i > 0)
            {
                auto [pr, pc] = path[i - 1];
                CHECK(std::abs(r - pr) + std::abs(c - pc) == 1);       // 步步相邻
            }
        }
    }

    // ========== Hybrid A* ==========
    {
        HybridParams params;
        VehicleState vstart{1.5, 1.5, 0.0};
        VehicleState vgoal{18.5, 10.5, 0.0};
        auto res = hybrid_astar_search(map, vstart, vgoal, params);

        CHECK(res.success);
        CHECK(res.path.size() > 1);
        if (res.success && res.path.size() > 1)
        {
            // 起点、终点正确
            CHECK(std::hypot(res.path.front().x - vstart.x, res.path.front().y - vstart.y) < 1e-9);
            CHECK(std::hypot(res.path.back().x - vgoal.x, res.path.back().y - vgoal.y)
                  < params.goal_xy_tol);

            const double max_dtheta =
                params.step_size / params.wheelbase *
                std::tan(std::atan(params.wheelbase / params.min_turning_radius)) + 1e-6;

            for (size_t i = 0; i < res.path.size(); ++i)
            {
                const auto& s = res.path[i];
                CHECK(!map.isObstacle(static_cast<int>(std::floor(s.y)),
                                      static_cast<int>(std::floor(s.x))));   // 不穿墙
                if (i > 0)
                {
                    double dtheta = std::abs(angle_diff(s.theta, res.path[i - 1].theta));
                    double dist = std::hypot(s.x - res.path[i - 1].x, s.y - res.path[i - 1].y);
                    // 每段弧长不超过一步仿真长度 + 解析扩展的小步
                    CHECK(dist < params.step_size + 1e-6 || dtheta < max_dtheta);
                }
            }
        }
    }

    if (failures == 0) { std::cout << "所有测试通过\n"; return 0; }
    std::cerr << failures << " 项测试失败\n";
    return 1;
}

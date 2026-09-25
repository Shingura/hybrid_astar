#include "hybrid_astar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace
{

constexpr double PI     = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double INF    = std::numeric_limits<double>::infinity();

// 把角度规约到 [0, 2π)
double normalize_angle(double a)
{
    while (a < 0.0)      a += TWO_PI;
    while (a >= TWO_PI)  a -= TWO_PI;
    return a;
}

// 带符号角度差 a - b，规约到 (-π, π]
double angle_diff(double a, double b)
{
    double d = normalize_angle(a - b);
    if (d > PI) d -= TWO_PI;
    return d;
}

// ============================================================
// 车辆运动学：自行车模型
//   x'     = v * cos(theta)
//   y'     = v * sin(theta)
//   theta' = v / L * tan(steer)
// 给定当前状态、方向盘转角 steer、行驶弧长 length（正=前进，负=倒车），
// 用若干小步数值积分求新状态。
// ============================================================
VehicleState simulate(const VehicleState& s, double steer, double length, double wheelbase)
{
    VehicleState r = s;
    const int sub_steps = 5;
    const double dl = length / sub_steps;
    for (int i = 0; i < sub_steps; ++i)
    {
        r.x     += dl * std::cos(r.theta);
        r.y     += dl * std::sin(r.theta);
        r.theta += dl / wheelbase * std::tan(steer);
    }
    r.theta = normalize_angle(r.theta);
    return r;
}

// 碰撞检查：把车辆当质点，检查所在格子是否是墙
// （工程做法是把障碍按车半径膨胀，本实现为教学简化）
bool collide(const GridMap& map, const VehicleState& s)
{
    return map.isObstacle(static_cast<int>(std::floor(s.y)),
                          static_cast<int>(std::floor(s.x)));
}

// 检查从 a 沿弧开到 b 的过程中是否撞墙（中间取若干采样点）
bool collide_along(const GridMap& map, const VehicleState& a, const VehicleState& b)
{
    const int samples = 6;
    for (int i = 1; i <= samples; ++i)
    {
        double t = static_cast<double>(i) / samples;
        VehicleState mid{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, 0.0 };
        if (collide(map, mid)) return true;
    }
    return false;
}

// 搜索节点：连续状态 + 代价 + 父节点下标（节点池里的位置）
struct HNode
{
    double x, y, theta;
    double g      = INF;    // cost_so_far
    double f      = INF;    // g + h
    double steer  = 0.0;    // 到达本节点时用的方向盘转角（算换向惩罚用）
    int    parent = -1;     // 节点池下标
};

// 优先队列元素：(f, 节点池下标)，f 最小先出
using OpenEntry = std::pair<double, int>;
struct Greater
{
    bool operator()(const OpenEntry& a, const OpenEntry& b) const { return a.first > b.first; }
};

// ============================================================
// 启发函数之一：遵守障碍、忽略车辆约束
// 从终点出发在栅格上跑一次 Dijkstra，得到每个格子到终点的真实绕行距离。
// 整次搜索只算一次，之后查表即可。
// ============================================================
std::vector<double> grid_distances_from_goal(const GridMap& map, int goal_row, int goal_col)
{
    const int rows = map.get_row_count(), cols = map.get_col_count();
    std::vector<double> dist(static_cast<size_t>(rows) * cols, INF);

    using Entry = std::pair<double, int>;   // (距离, 格子下标)
    std::priority_queue<Entry, std::vector<Entry>, Greater> pq;

    auto index = [cols](int r, int c) { return r * cols + c; };
    dist[index(goal_row, goal_col)] = 0.0;
    pq.push({0.0, index(goal_row, goal_col)});

    const int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    while (!pq.empty())
    {
        auto [d, i] = pq.top(); pq.pop();
        if (d > dist[i]) continue;   // 过期副本
        int r = i / cols, c = i % cols;
        for (auto& [dr, dc] : dirs)
        {
            int nr = r + dr, nc = c + dc;
            if (!map.inBound(nr, nc) || map.isObstacle(nr, nc)) continue;
            if (d + 1.0 < dist[index(nr, nc)])
            {
                dist[index(nr, nc)] = d + 1.0;
                pq.push({d + 1.0, index(nr, nc)});
            }
        }
    }
    return dist;
}

// ============================================================
// 解析扩展：纯追踪（pure pursuit）尝试从当前状态直接开到终点
// 每小步朝终点方向修正方向盘，能无碰撞开到就算成功。
// 是 Reeds-Shepp 直连的简化替代（见 docs/hybrid_astar.md）。
// ============================================================
bool try_analytic_expansion(const GridMap& map, const HybridParams& P, double max_steer,
                            const VehicleState& from, const VehicleState& goal,
                            std::vector<VehicleState>& shot)
{
    shot.clear();
    VehicleState s = from;
    for (int k = 0; k < 500; ++k)
    {
        double dx = goal.x - s.x, dy = goal.y - s.y;
        if (std::hypot(dx, dy) < P.goal_xy_tol) return true;

        // 终点在车身侧后方时纯追踪很难开到，直接放弃这次尝试
        double err = angle_diff(std::atan2(dy, dx), s.theta);
        if (std::abs(err) > PI * 0.75) return false;

        double steer = std::clamp(2.0 * err, -max_steer, max_steer);
        VehicleState next = simulate(s, steer, 0.2, P.wheelbase);
        if (collide(map, next) || collide_along(map, s, next)) return false;
        s = next;
        shot.push_back(s);
    }
    return false;
}

} // namespace

HybridResult hybrid_astar_search(const GridMap& map,
                                 VehicleState start, VehicleState goal,
                                 const HybridParams& P)
{
    const int rows = map.get_row_count(), cols = map.get_col_count(), bins = P.heading_bins;

    // 最大方向盘转角：由最小转弯半径决定，tan(δmax) = L / R
    const double max_steer = std::atan(P.wheelbase / P.min_turning_radius);

    // ---- 离散化记账：连续状态 → (格行, 格列, 朝向档) 桶 ----
    auto bucket = [&](const VehicleState& s) {
        int r  = static_cast<int>(std::floor(s.y));
        int c  = static_cast<int>(std::floor(s.x));
        int b  = static_cast<int>(normalize_angle(s.theta) / TWO_PI * bins);
        b = std::clamp(b, 0, bins - 1);
        return (r * cols + c) * bins + b;
    };
    std::vector<double> best_cost(static_cast<size_t>(rows) * cols * bins, INF);

    // ---- 启发：栅格 Dijkstra 距离（含障碍）与欧氏距离取大 ----
    const int goal_row = static_cast<int>(std::floor(goal.y));
    const int goal_col = static_cast<int>(std::floor(goal.x));
    std::vector<double> grid_dist = grid_distances_from_goal(map, goal_row, goal_col);
    auto heuristic = [&](const VehicleState& s) {
        int r = static_cast<int>(std::floor(s.y));
        int c = static_cast<int>(std::floor(s.x));
        double h_grid = map.inBound(r, c) ? grid_dist[static_cast<size_t>(r) * cols + c] : INF;
        double h_euclid = std::hypot(goal.x - s.x, goal.y - s.y);
        return std::max(h_grid, h_euclid);
    };

    // ---- open list / 节点池 ----
    std::vector<HNode> pool;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, Greater> open;

    pool.push_back(HNode{start.x, start.y, start.theta, 0.0, heuristic(start), 0.0, -1});
    best_cost[bucket(start)] = 0.0;
    open.push({pool[0].f, 0});

    HybridResult result;
    int goal_index = -1;

    while (!open.empty())
    {
        auto [f, i] = open.top(); open.pop();
        const HNode cur = pool[i];
        if (cur.g > best_cost[bucket({cur.x, cur.y, cur.theta})] + 1e-9) continue;   // 过期副本
        ++result.expanded_nodes;

        // 到达终点？
        double goal_dist = std::hypot(goal.x - cur.x, goal.y - cur.y);
        if (goal_dist < P.goal_xy_tol && std::abs(angle_diff(goal.theta, cur.theta)) < P.goal_theta_tol)
        {
            goal_index = i;
            break;
        }

        // 定期尝试解析扩展直连终点
        if (result.expanded_nodes % P.analytic_expansion_interval == 0)
        {
            std::vector<VehicleState> shot;
            const VehicleState cur_state{cur.x, cur.y, cur.theta};
            if (try_analytic_expansion(map, P, max_steer, cur_state, goal, shot))
            {
                goal_index = i;
                // 把解析段接进节点池，串到回溯链上
                for (const auto& s : shot)
                {
                    pool.push_back(HNode{s.x, s.y, s.theta, 0.0, 0.0, 0.0, goal_index});
                    goal_index = static_cast<int>(pool.size()) - 1;
                }
                break;
            }
        }

        // ---- 扩展：采样方向盘转角 × 前进/倒车 ----
        for (int s = 0; s < P.steering_samples; ++s)
        {
            double steer = -max_steer + 2.0 * max_steer * s / (P.steering_samples - 1);
            for (int dir : {1, -1})
            {
                if (dir < 0 && !P.allow_reverse) continue;

                VehicleState next = simulate({cur.x, cur.y, cur.theta}, steer,
                                             dir * P.step_size, P.wheelbase);
                if (collide(map, next) || collide_along(map, {cur.x, cur.y, 0.0}, next))
                    continue;

                // 代价 = 弧长（倒车加倍）+ 方向盘变化惩罚
                double g = cur.g + P.step_size * (dir < 0 ? P.reverse_penalty : 1.0)
                                 + P.steer_change_penalty * std::abs(steer - cur.steer);

                // 松弛：同桶内只有成绩更好才更新
                int b = bucket(next);
                if (g >= best_cost[b] - 1e-9) continue;
                best_cost[b] = g;

                pool.push_back(HNode{next.x, next.y, next.theta, g, g + heuristic(next), steer, i});
                open.push({g + heuristic(next), static_cast<int>(pool.size()) - 1});
            }
        }
    }

    // ---- 回溯 ----
    if (goal_index < 0) return result;
    result.success = true;
    for (int i = goal_index; i >= 0; i = pool[i].parent)
        result.path.push_back({pool[i].x, pool[i].y, pool[i].theta});
    std::reverse(result.path.begin(), result.path.end());

    for (size_t i = 1; i < result.path.size(); ++i)
        result.path_length += std::hypot(result.path[i].x - result.path[i - 1].x,
                                         result.path[i].y - result.path[i - 1].y);
    return result;
}

#pragma once

#include "grid_map.h"

#include <vector>

// 车辆的连续状态：位置 (x, y) + 车头朝向 theta
// 注意坐标对应关系：x = 列(col) 方向，y = 行(row) 方向，与栅格地图一致
// 格子 (row, col) 的中心在连续坐标里是 (col + 0.5, row + 0.5)
struct VehicleState
{
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;   // 弧度，0 = 朝 +x（右），逆时针为正
};

// hybrid A* 的可调参数，全部给了默认值，可以直接用
struct HybridParams
{
    // --- 运动学 ---
    double wheelbase           = 1.0;   // 轴距 L
    double min_turning_radius  = 2.0;   // 最小转弯半径（决定最大方向盘转角）
    double step_size           = 0.5;   // 每次扩展仿真多长的弧

    // --- 扩展 ---
    int    steering_samples    = 5;     // 方向盘转角采样个数（含直行），奇数为宜
    bool   allow_reverse       = true;  // 是否允许倒车

    // --- 代价惩罚 ---
    double reverse_penalty        = 2.0;   // 倒车代价倍率
    double steer_change_penalty   = 0.5;   // 方向盘转角变化惩罚（每弧度）

    // --- 离散化（记账用） ---
    int    heading_bins = 72;   // 朝向离散成多少档（72 = 每 5° 一档）

    // --- 终点判定 ---
    double goal_xy_tol    = 0.6;   // 位置容差（格）
    double goal_theta_tol = 0.4;   // 朝向容差（弧度）

    // --- 解析扩展 ---
    int analytic_expansion_interval = 20;  // 每扩展多少个节点尝试一次直连终点
};

struct HybridResult
{
    bool success = false;
    std::vector<VehicleState> path;   // 从起点到终点的连续状态序列
    int expanded_nodes = 0;           // 扩展过的节点数（性能参考）
    double path_length = 0.0;         // 路径总弧长
};

// hybrid A* 主入口：在 map 上为车辆搜索一条从 start 到 goal 的可行驶路径
HybridResult hybrid_astar_search(const GridMap& map,
                                 VehicleState start,
                                 VehicleState goal,
                                 const HybridParams& params = HybridParams{});

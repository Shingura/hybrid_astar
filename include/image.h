#pragma once

#include "grid_map.h"

#include <string>
#include <utility>
#include <vector>

struct Color
{
    unsigned char r, g, b;
};

// 极简画布：内存里存 RGB 像素，能保存成 PPM 图片（零依赖）
class Image
{
public:
    Image(int width, int height, Color bg = {255, 255, 255});

    int width()  const { return width_; }
    int height() const { return height_; }

    void setPixel(int x, int y, Color c);
    void fillRect(int x, int y, int w, int h, Color c);
    void drawLine(int x0, int y0, int x1, int y1, Color c);   // Bresenham 直线

    bool save(const std::string& path) const;   // 保存为二进制 PPM (P6)

private:
    int width_  = 0;
    int height_ = 0;
    std::vector<unsigned char> pixels_;   // RGBRGB...，下标 = (y * width + x) * 3
};

// 把栅格地图渲染成图片：墙黑、空地白，每格 scale 像素
Image render_map(const GridMap& map, int scale);

// 在图上画栅格路径（来自 A* 的 (row, col) 序列），整格涂色
void draw_grid_path(Image& img, const std::vector<std::pair<int, int>>& path,
                    int scale, Color c);

// 在图上画连续路径（来自 hybrid A* 的 (x, y) 序列，单位是"格"），折线
void draw_polyline(Image& img, const std::vector<std::pair<double, double>>& points,
                   int scale, Color c);

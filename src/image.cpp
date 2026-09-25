#include "image.h"

#include <cstdlib>
#include <fstream>

Image::Image(int width, int height, Color bg)
    : width_(width), height_(height), pixels_(static_cast<size_t>(width) * height * 3)
{
    fillRect(0, 0, width, height, bg);
}

void Image::setPixel(int x, int y, Color c)
{
    if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
    size_t i = (static_cast<size_t>(y) * width_ + x) * 3;
    pixels_[i] = c.r; pixels_[i + 1] = c.g; pixels_[i + 2] = c.b;
}

void Image::fillRect(int x, int y, int w, int h, Color c)
{
    for (int dy = 0; dy < h; ++dy)
        for (int dx = 0; dx < w; ++dx)
            setPixel(x + dx, y + dy, c);
}

void Image::drawLine(int x0, int y0, int x1, int y1, Color c)
{
    // 经典 Bresenham 直线算法
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true)
    {
        setPixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

bool Image::save(const std::string& path) const
{
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << "P6\n" << width_ << ' ' << height_ << "\n255\n";
    out.write(reinterpret_cast<const char*>(pixels_.data()),
              static_cast<std::streamsize>(pixels_.size()));
    return out.good();
}

Image render_map(const GridMap& map, int scale)
{
    Image img(map.get_col_count() * scale, map.get_row_count() * scale);
    for (int row = 0; row < map.get_row_count(); ++row)
        for (int col = 0; col < map.get_col_count(); ++col)
            if (map.isObstacle(row, col))
                img.fillRect(col * scale, row * scale, scale, scale, {40, 40, 40});
    return img;
}

void draw_grid_path(Image& img, const std::vector<std::pair<int, int>>& path,
                    int scale, Color c)
{
    for (const auto& [row, col] : path)
        img.fillRect(col * scale + 1, row * scale + 1, scale - 2, scale - 2, c);
}

void draw_polyline(Image& img, const std::vector<std::pair<double, double>>& points,
                   int scale, Color c)
{
    for (size_t i = 1; i < points.size(); ++i)
    {
        auto [x0, y0] = points[i - 1];
        auto [x1, y1] = points[i];
        img.drawLine(static_cast<int>(x0 * scale), static_cast<int>(y0 * scale),
                     static_cast<int>(x1 * scale), static_cast<int>(y1 * scale), c);
    }
}

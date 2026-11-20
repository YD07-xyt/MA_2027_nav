#pragma once

#include "map/core/grid_map.hpp"

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <string>

namespace test_utils {

/**
 * 黑白图片 -> grid_map::GridMap。
 *
 * 坐标约定:
 *  - 地图原点在图片左下角,x 轴向右(图片列方向),y 轴向上(图片行的反方向)。
 *  - 栅格 (ix, iy) 对应像素 (col = ix, row = H - 1 - iy),一个像素一个栅格。
 *  - 灰度 < occupied_threshold 视为障碍(黑色)。
 */
class Cv2Map {
public:
    struct Options {
        double resolution = 0.05;    // 每像素对应的米数
        int occupied_threshold = 128; // 灰度小于该值为障碍
        double inflate_radius = 0.0;  // >0 时对障碍做圆形膨胀(m)
    };

    explicit Cv2Map(const std::string& image_path): Cv2Map(image_path, Options {}) {}

    Cv2Map(const std::string& image_path, const Options& opt): opt_(opt), path_(image_path) {
        if (opt_.resolution <= 0.0) throw std::invalid_argument("Cv2Map: resolution must be > 0");

        gray_ = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if (gray_.empty()) throw std::runtime_error("Cv2Map: failed to read image: " + image_path);

        const int w = gray_.cols;
        const int h = gray_.rows;

        // 尺寸略小于 w*res,避免浮点误差让 ceil(size/res) 多出一个栅格。
        const double size_x = (w - 1e-3) * opt_.resolution;
        const double size_y = (h - 1e-3) * opt_.resolution;
        map_.init(0.0, size_x, 0.0, size_y, opt_.resolution);

        const Eigen::Vector2i n = map_.getVoxelNum();
        if (n.x() != w || n.y() != h) {
            throw std::runtime_error("Cv2Map: grid size mismatch with image size");
        }

        grid_map::RowMatrixXi occ(n.x(), n.y());
        occupied_count_ = 0;
        for (int ix = 0; ix < n.x(); ++ix) {
            for (int iy = 0; iy < n.y(); ++iy) {
                const int row = h - 1 - iy;
                const bool occupied = gray_.at<uint8_t>(row, ix) < opt_.occupied_threshold;
                occ(ix, iy) = occupied ? 1 : 0;
                occupied_count_ += occupied ? 1 : 0;
            }
        }
        map_.setMap(occ); // 内部计算 ESDF
        if (opt_.inflate_radius > 0.0) map_.inflate(opt_.inflate_radius);
    }

    auto grid_map() const -> const grid_map::GridMap& {
        return map_;
    }
    auto gray() const -> const cv::Mat& {
        return gray_;
    }
    auto color() const -> cv::Mat {
        cv::Mat bgr;
        cv::cvtColor(gray_, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    auto path() const -> const std::string& {
        return path_;
    }
    auto width() const -> int {
        return gray_.cols;
    }
    auto height() const -> int {
        return gray_.rows;
    }
    auto resolution() const -> double {
        return opt_.resolution;
    }
    auto occupied_ratio() const -> double {
        return static_cast<double>(occupied_count_) / (static_cast<double>(width()) * height());
    }

    // 像素中心 -> 世界坐标(m)
    auto pixel_to_world(int col, int row) const -> Eigen::Vector2d {
        return {(col + 0.5) * opt_.resolution, (height() - row - 0.5) * opt_.resolution};
    }

    // 世界坐标 -> 放大 scale 倍后图片上的像素坐标(用于绘图)
    auto world_to_pixel(const Eigen::Vector2d& p, double scale = 1.0) const -> cv::Point {
        const double u = p.x() / opt_.resolution * scale;
        const double v = (height() - p.y() / opt_.resolution) * scale;
        return {static_cast<int>(std::lround(u)), static_cast<int>(std::lround(v))};
    }

private:
    Options opt_;
    std::string path_;
    cv::Mat gray_;
    grid_map::GridMap map_;
    int occupied_count_ = 0;
};

} // namespace test_utils

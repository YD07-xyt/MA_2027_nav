// 规划可视化模块：测试程序共用的 OpenCV 绘图工具（路径、点、标签、分组柱状图、拼图、保存）。
// 坐标换算依赖 Cv2Map（世界坐标 -> 放大后的图片像素）。
#pragma once

#include "cv2map.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace test_utils::viz {

// BGR 常用色
inline const cv::Scalar kRed(40, 40, 230);
inline const cv::Scalar kBlue(230, 120, 20);
inline const cv::Scalar kGreen(0, 180, 0);
inline const cv::Scalar kMagenta(200, 0, 200);
inline const cv::Scalar kOrange(0, 140, 255);
inline const cv::Scalar kGray(170, 170, 170);
inline const cv::Scalar kBlack(0, 0, 0);
inline const cv::Scalar kWhite(255, 255, 255);

inline auto strf(const char* f, double v) -> std::string {
    char buf[64];
    std::snprintf(buf, sizeof(buf), f, v);
    return buf;
}

// 原图放大 scale 倍的彩色底图
inline auto base_image(const Cv2Map& m, double scale) -> cv::Mat {
    cv::Mat img;
    cv::resize(m.color(), img, cv::Size(), scale, scale, cv::INTER_NEAREST);
    return img;
}

inline auto to_pixels(const Cv2Map& m, const std::vector<Eigen::Vector2d>& path, double scale)
    -> std::vector<cv::Point> {
    std::vector<cv::Point> pts;
    pts.reserve(path.size());
    for (const auto& p: path) pts.push_back(m.world_to_pixel(p, scale));
    return pts;
}

// 折线；mark_waypoints 时在每个点上画空心圆
inline auto draw_path(cv::Mat& img, const Cv2Map& m, const std::vector<Eigen::Vector2d>& path,
                      const cv::Scalar& color, int thickness, bool mark_waypoints, double scale) -> void {
    const auto pts = to_pixels(m, path, scale);
    if (pts.size() >= 2) cv::polylines(img, pts, false, color, thickness, cv::LINE_AA);
    if (mark_waypoints) {
        for (const auto& pt: pts) cv::circle(img, pt, thickness + 2, color, 1, cv::LINE_AA);
    }
}

// 实心点
inline auto draw_points(cv::Mat& img, const Cv2Map& m, const std::vector<Eigen::Vector2d>& pts_w,
                        const cv::Scalar& color, int radius, double scale) -> void {
    for (const auto& pt: to_pixels(m, pts_w, scale)) cv::circle(img, pt, radius, color, cv::FILLED, cv::LINE_AA);
}

inline auto draw_start_goal(cv::Mat& img, const Cv2Map& m, const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                            double scale) -> void {
    cv::circle(img, m.world_to_pixel(start, scale), 7, kGreen, cv::FILLED, cv::LINE_AA);
    cv::circle(img, m.world_to_pixel(goal, scale), 7, kMagenta, cv::FILLED, cv::LINE_AA);
}

// 白底文字标签
inline auto draw_label(cv::Mat& img, const std::string& text, cv::Point org, const cv::Scalar& color) -> void {
    int base = 0;
    const auto sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
    cv::rectangle(img, org + cv::Point(-3, -sz.height - 4), org + cv::Point(sz.width + 3, base + 2), kWhite,
                  cv::FILLED);
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
}

inline auto draw_frame(cv::Mat& img) -> void {
    cv::rectangle(img, {0, 0}, {img.cols - 1, img.rows - 1}, cv::Scalar(120, 120, 120), 2);
}

// 最多 4 张拼成 2x2（不足用白图补齐）；单张直接返回
inline auto mosaic_2x2(std::vector<cv::Mat> panels) -> cv::Mat {
    if (panels.empty()) return {};
    if (panels.size() == 1) return panels[0];
    panels.resize(std::min<size_t>(4, panels.size()));
    while (panels.size() < 4) panels.push_back(cv::Mat(panels[0].size(), panels[0].type(), kWhite));
    cv::Mat top, bottom, out;
    cv::hconcat(panels[0], panels[1], top);
    cv::hconcat(panels[2], panels[3], bottom);
    cv::vconcat(top, bottom, out);
    return out;
}

// 柱状图的一组数据
struct Series {
    std::string name;
    cv::Scalar color;
    std::vector<double> values; // 值 <=0 或 NaN 视为失败，画 "x"
};

// 分组柱状图：每个标签一组，每组 series.size() 根柱
inline auto draw_grouped_bars(const std::string& title, const std::vector<std::string>& labels,
                              const std::vector<Series>& series, bool log_scale, const std::string& y_unit, int width,
                              int height, bool annotate) -> cv::Mat {
    cv::Mat img(height, width, CV_8UC3, kWhite);
    const int left = 90, right = 30, top = 70, bottom = 60;
    const int pw = width - left - right;
    const int ph = height - top - bottom;

    auto valid = [](double v) { return std::isfinite(v) && v > 0.0; };
    double vmin = std::numeric_limits<double>::max(), vmax = 0.0;
    for (const auto& s: series) {
        for (double v: s.values) {
            if (!valid(v)) continue;
            vmin = std::min(vmin, v);
            vmax = std::max(vmax, v);
        }
    }
    if (vmax <= 0.0) vmin = vmax = 1.0;

    double lo, hi;
    if (log_scale) {
        lo = std::floor(std::log10(vmin));
        hi = std::ceil(std::log10(vmax));
        if (hi <= lo) hi = lo + 1.0;
    } else {
        lo = 0.0;
        hi = vmax * 1.15;
    }
    auto to_y = [&](double v) {
        const double t = log_scale ? (std::log10(v) - lo) / (hi - lo) : (v - lo) / (hi - lo);
        return top + ph - static_cast<int>(std::lround(std::clamp(t, 0.0, 1.0) * ph));
    };

    // 网格与纵轴刻度
    std::vector<double> ticks;
    if (log_scale) {
        for (double e = lo; e <= hi + 1e-9; e += 1.0) ticks.push_back(std::pow(10.0, e));
    } else {
        for (int i = 0; i <= 5; ++i) ticks.push_back(hi * i / 5.0);
    }
    for (double t: ticks) {
        const int y = log_scale ? to_y(t) : to_y(std::max(t, 1e-12));
        cv::line(img, {left, y}, {left + pw, y}, cv::Scalar(225, 225, 225), 1);
        const std::string s = (t >= 1.0 || t == 0.0) ? strf("%g", t) : strf("%.0e", t);
        cv::putText(img, s, {8, y + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
    }
    cv::line(img, {left, top}, {left, top + ph}, kBlack, 1);
    cv::line(img, {left, top + ph}, {left + pw, top + ph}, kBlack, 1);
    cv::putText(img, y_unit + (log_scale ? " (log)" : ""), {8, top - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(60, 60, 60), 1, cv::LINE_AA);

    // 柱
    const int n = static_cast<int>(labels.size());
    const int ns = std::max<int>(1, static_cast<int>(series.size()));
    const double group_w = n > 0 ? static_cast<double>(pw) / n : pw;
    const int bar_w = std::max(2, static_cast<int>(group_w * 0.7 / ns));
    for (int i = 0; i < n; ++i) {
        const int cx = left + static_cast<int>(group_w * (i + 0.5));
        const int x_begin = cx - bar_w * ns / 2;
        for (int k = 0; k < static_cast<int>(series.size()); ++k) {
            const auto& s = series[k];
            const double v = i < static_cast<int>(s.values.size()) ? s.values[i] : NAN;
            const int x0 = x_begin + k * bar_w;
            if (!valid(v)) {
                cv::putText(img, "x", {x0 + bar_w / 2 - 4, top + ph - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.5, s.color, 1);
                continue;
            }
            const int y = to_y(v);
            cv::rectangle(img, {x0, y}, {x0 + bar_w - 1, top + ph}, s.color, cv::FILLED);
            if (annotate) {
                const std::string t = v >= 1.0 ? strf("%.2f", v) : strf("%.3f", v);
                cv::putText(img, t, {x0, y - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(30, 30, 30), 1,
                            cv::LINE_AA);
            }
        }
        int base = 0;
        const auto sz = cv::getTextSize(labels[i], cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &base);
        cv::putText(img, labels[i], {cx - sz.width / 2, top + ph + 20}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
    }

    // 标题与图例（从右往左排）
    cv::putText(img, title, {left, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, kBlack, 1, cv::LINE_AA);
    int lx = width - right;
    for (int k = static_cast<int>(series.size()) - 1; k >= 0; --k) {
        int base = 0;
        const auto sz = cv::getTextSize(series[k].name, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
        lx -= sz.width + 30;
        cv::rectangle(img, {lx, 40}, {lx + 14, 52}, series[k].color, cv::FILLED);
        cv::putText(img, series[k].name, {lx + 20, 52}, cv::FONT_HERSHEY_SIMPLEX, 0.5, kBlack, 1, cv::LINE_AA);
    }
    return img;
}

// 标量 -> 颜色（蓝 -> 绿 -> 红），t 取 [0,1]
inline auto heat_color(double t) -> cv::Scalar {
    t = std::clamp(t, 0.0, 1.0);
    if (t < 0.5) {
        const double k = t / 0.5;
        return {255.0 * (1.0 - k), 200.0 * k, 0.0};
    }
    const double k = (t - 0.5) / 0.5;
    return {0.0, 200.0 * (1.0 - k), 255.0 * k};
}

// 按标量着色的折线（每段颜色取两端标量均值），values 归一化到 [vmin, vmax]
inline auto draw_colored_path(cv::Mat& img, const Cv2Map& m, const std::vector<Eigen::Vector2d>& path,
                              const std::vector<double>& values, double vmin, double vmax, int thickness, double scale)
    -> void {
    const auto pts = to_pixels(m, path, scale);
    const double span = vmax > vmin ? vmax - vmin : 1.0;
    for (size_t i = 1; i < pts.size() && i < values.size(); ++i) {
        const double v = 0.5 * (values[i - 1] + values[i]);
        cv::line(img, pts[i - 1], pts[i], heat_color((v - vmin) / span), thickness, cv::LINE_AA);
    }
}

// 竖直色条图例
inline auto draw_colorbar(cv::Mat& img, cv::Point org, int h, double vmin, double vmax, const std::string& unit)
    -> void {
    const int w = 14;
    cv::rectangle(img, org + cv::Point(-4, -18), org + cv::Point(w + 60, h + 8), kWhite, cv::FILLED);
    for (int y = 0; y < h; ++y) {
        cv::line(img, org + cv::Point(0, y), org + cv::Point(w, y), heat_color(1.0 - static_cast<double>(y) / h), 1);
    }
    cv::rectangle(img, org, org + cv::Point(w, h), kBlack, 1);
    cv::putText(img, strf("%.1f", vmax), org + cv::Point(w + 4, 10), cv::FONT_HERSHEY_SIMPLEX, 0.45, kBlack, 1,
                cv::LINE_AA);
    cv::putText(img, strf("%.1f", vmin), org + cv::Point(w + 4, h), cv::FONT_HERSHEY_SIMPLEX, 0.45, kBlack, 1,
                cv::LINE_AA);
    cv::putText(img, unit, org + cv::Point(0, -5), cv::FONT_HERSHEY_SIMPLEX, 0.45, kBlack, 1, cv::LINE_AA);
}

// 折线图的一条曲线
struct Curve {
    std::string name;
    cv::Scalar color;
    std::vector<double> x;
    std::vector<double> y;
};

// 水平参考线（如限幅）
struct HLine {
    std::string name;
    cv::Scalar color;
    double y;
};

// 折线图：多条曲线 + 水平参考线（虚线）
inline auto draw_line_chart(const std::string& title, const std::vector<Curve>& curves, const std::vector<HLine>& refs,
                            const std::string& x_unit, const std::string& y_unit, int width, int height) -> cv::Mat {
    cv::Mat img(height, width, CV_8UC3, kWhite);
    const int left = 70, right = 20, top = 60, bottom = 45;
    const int pw = width - left - right;
    const int ph = height - top - bottom;

    double xmin = std::numeric_limits<double>::max(), xmax = std::numeric_limits<double>::lowest();
    double ymin = std::numeric_limits<double>::max(), ymax = std::numeric_limits<double>::lowest();
    for (const auto& c: curves) {
        for (size_t i = 0; i < c.x.size() && i < c.y.size(); ++i) {
            if (!std::isfinite(c.x[i]) || !std::isfinite(c.y[i])) continue;
            xmin = std::min(xmin, c.x[i]);
            xmax = std::max(xmax, c.x[i]);
            ymin = std::min(ymin, c.y[i]);
            ymax = std::max(ymax, c.y[i]);
        }
    }
    for (const auto& r: refs) {
        ymin = std::min(ymin, r.y);
        ymax = std::max(ymax, r.y);
    }
    if (xmax <= xmin) xmax = xmin + 1.0;
    if (ymax <= ymin) ymax = ymin + 1.0;
    const double ypad = 0.08 * (ymax - ymin);
    ymin -= ypad;
    ymax += ypad;

    auto to_px = [&](double x, double y) {
        return cv::Point(left + static_cast<int>(std::lround((x - xmin) / (xmax - xmin) * pw)),
                         top + ph - static_cast<int>(std::lround((y - ymin) / (ymax - ymin) * ph)));
    };

    for (int i = 0; i <= 5; ++i) {
        const double yv = ymin + (ymax - ymin) * i / 5.0;
        const int y = to_px(xmin, yv).y;
        cv::line(img, {left, y}, {left + pw, y}, cv::Scalar(225, 225, 225), 1);
        cv::putText(img, strf("%.2f", yv), {6, y + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(60, 60, 60), 1,
                    cv::LINE_AA);
        const double xv = xmin + (xmax - xmin) * i / 5.0;
        const int x = to_px(xv, ymin).x;
        cv::putText(img, strf("%.1f", xv), {x - 12, top + ph + 18}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
    }
    cv::rectangle(img, {left, top}, {left + pw, top + ph}, kBlack, 1);
    cv::putText(img, x_unit, {left + pw - 30, top + ph + 38}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kBlack, 1, cv::LINE_AA);
    cv::putText(img, y_unit, {6, top - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kBlack, 1, cv::LINE_AA);

    // 参考线（虚线）
    for (const auto& r: refs) {
        const int y = to_px(xmin, r.y).y;
        for (int x = left; x < left + pw; x += 12) cv::line(img, {x, y}, {std::min(x + 6, left + pw), y}, r.color, 1);
    }
    for (const auto& c: curves) {
        std::vector<cv::Point> pts;
        for (size_t i = 0; i < c.x.size() && i < c.y.size(); ++i) {
            if (std::isfinite(c.x[i]) && std::isfinite(c.y[i])) pts.push_back(to_px(c.x[i], c.y[i]));
        }
        if (pts.size() >= 2) cv::polylines(img, pts, false, c.color, 2, cv::LINE_AA);
    }

    cv::putText(img, title, {left, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.55, kBlack, 1, cv::LINE_AA);
    int lx = left;
    auto legend = [&](const std::string& name, const cv::Scalar& color, bool dashed) {
        if (dashed) cv::line(img, {lx, 42}, {lx + 16, 42}, color, 1);
        else cv::line(img, {lx, 42}, {lx + 16, 42}, color, 2);
        cv::putText(img, name, {lx + 20, 47}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kBlack, 1, cv::LINE_AA);
        int base = 0;
        lx += 30 + cv::getTextSize(name, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &base).width;
    };
    for (const auto& c: curves) legend(c.name, c.color, false);
    for (const auto& r: refs) legend(r.name, r.color, true);
    return img;
}

// 竖直拼接（宽度不同时右侧补白）
inline auto vstack(const std::vector<cv::Mat>& imgs) -> cv::Mat {
    int w = 0;
    for (const auto& m: imgs) w = std::max(w, m.cols);
    std::vector<cv::Mat> padded;
    for (const auto& m: imgs) {
        if (m.empty()) continue;
        cv::Mat p;
        cv::copyMakeBorder(m, p, 0, 0, 0, w - m.cols, cv::BORDER_CONSTANT, kWhite);
        padded.push_back(p);
    }
    if (padded.empty()) return {};
    cv::Mat out;
    cv::vconcat(padded, out);
    return out;
}

inline auto save(const std::string& dir, const std::string& file, const cv::Mat& img) -> void {
    if (img.empty()) return;
    const std::string path = dir + file;
    if (cv::imwrite(path, img)) std::printf("  saved %s\n", path.c_str());
    else std::printf("  FAILED to save %s\n", path.c_str());
}

} // namespace test_utils::viz

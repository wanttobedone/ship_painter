#include "ship_painter/bspline.h"
#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <iostream>

namespace ship_painter {

void BSpline::fitFromContour(
    const std::vector<Eigen::Vector3d>& contour_points,
    const std::vector<Eigen::Vector3d>& normals,
    double cruise_speed,
    bool closed) {
    
    if (contour_points.empty()) {
        ROS_ERROR("Cannot fit B-spline: empty contour");
        return;
    }
    
    control_points_ = contour_points;
    normals_ = normals;
    
    // 1. 处理闭合
    if (closed && contour_points.size() > 1) {
        control_points_.push_back(contour_points[0]);
        normals_.push_back(normals[0]);
    }
    
    // 2. 计算总长度
    double total_length = 0.0;
    for (size_t i = 1; i < control_points_.size(); i++) {
        total_length += (control_points_[i] - control_points_[i-1]).norm();
    }
    
    // 3. 计算总时间
    if (cruise_speed < 1e-3) cruise_speed = 1.0; // 保护
    total_time_ = total_length / cruise_speed;
    
    // 4. 生成节点向量
    generateChordLengthKnots(control_points_, total_time_);
    
    ROS_INFO("B-spline fitted: %zu pts, time=%.2f s, len=%.2f m",
             control_points_.size(), total_time_, total_length);
}

void BSpline::generateChordLengthKnots(
    const std::vector<Eigen::Vector3d>& points,
    double total_time) {
    
    int n = points.size(); 
    int m = n + degree_ + 1;
    
    knots_.clear();
    knots_.resize(m);
    
    // 起点重复节点
    for (int i = 0; i <= degree_; i++) knots_[i] = 0.0;
    
    // 弦长累加
    std::vector<double> cumulative_length(n, 0.0);
    double current_len = 0.0;
    for (int i = 1; i < n; i++) {
        current_len += (points[i] - points[i-1]).norm();
        cumulative_length[i] = current_len;
    }
    double total_len_geom = cumulative_length[n-1];
    
    // 中间节点
    for (int i = 1; i < n - degree_; i++) {
        // 映射到 [0, total_time]
        double u_norm = cumulative_length[i] / total_len_geom;
        knots_[i + degree_] = u_norm * total_time;
    }
    
    // 终点重复节点
    for (int i = m - degree_ - 1; i < m; i++) knots_[i] = total_time;
}

    // 辅助函数：计算 t 时刻所有非零基函数的值及其一阶、二阶导数
    // 返回值: bases[k][j] 表示第 j 个基函数的 k 阶导数 (k=0,1,2)
void BSpline::evaluateBasisDerivatives(double t, int span,
                                     std::vector<std::vector<double>>& bases) const {
    // 初始化：支持到3阶导数 (0,1,2,3)
    int max_deriv = std::min(3, degree_);  // 最多3阶，但不能超过B样条阶数
    bases.assign(max_deriv + 1, std::vector<double>(degree_ + 1, 0.0));

    std::vector<double> left(degree_ + 1, 0.0);
    std::vector<double> right(degree_ + 1, 0.0);
    std::vector<std::vector<double>> ndu(degree_ + 1, std::vector<double>(degree_ + 1));

    ndu[0][0] = 1.0;

    // 1. 计算所有基函数值 (N_i,p) - 三角形迭代
    for (int j = 1; j <= degree_; j++) {
        left[j] = t - knots_[span + 1 - j];
        right[j] = knots_[span + j] - t;
        double saved = 0.0;

        for (int r = 0; r < j; r++) {
            // 下三角部分
            ndu[j][r] = right[r + 1] + left[j - r];
            double temp = ndu[r][j - 1] / ndu[j][r];

            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    // 存入0阶导数 (原值)
    for (int j = 0; j <= degree_; j++) {
        bases[0][j] = ndu[j][degree_];
    }

    // 2. 计算导数
    // a[k][j] 存储中间变量
    std::vector<std::vector<double>> a(2, std::vector<double>(degree_ + 1));

    for (int r = 0; r <= degree_; r++) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;

        // 计算 k 阶导数（扩展到3阶）
        for (int k = 1; k <= max_deriv; k++) {
            double d = 0.0;
            int rk = r - k;
            int pk = degree_ - k;

            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            int j1 = (rk >= -1) ? 1 : -rk;
            int j2 = (r - 1 <= pk) ? k - 1 : degree_ - r;

            for (int j = j1; j <= j2; j++) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }

            bases[k][r] = d;
            std::swap(s1, s2);
        }
    }

    // 3. 修正导数系数 (乘以阶数因子)
    double coef = degree_;
    for (int k = 1; k <= max_deriv; k++) {
        for (int j = 0; j <= degree_; j++) {
            bases[k][j] *= coef;
        }
        coef *= (degree_ - k);
    }
}

Eigen::Vector3d BSpline::getPosition(double t) const {
    if (control_points_.empty()) return Eigen::Vector3d::Zero();
    t = std::max(0.0, std::min(t, total_time_));
    if (t >= total_time_) return control_points_.back();

    int span = findSpan(t);
    
    // 计算基函数
    std::vector<std::vector<double>> bases;
    evaluateBasisDerivatives(t, span, bases); // 只需0阶
    
    Eigen::Vector3d p(0, 0, 0);
    for (int i = 0; i <= degree_; i++) {
        p += control_points_[span - degree_ + i] * bases[0][i];
    }
    return p;
}

Eigen::Vector3d BSpline::getVelocity(double t) const {
    if (control_points_.size() < 2) return Eigen::Vector3d::Zero();
    t = std::max(0.0, std::min(t, total_time_));

    int span = findSpan(t);
    std::vector<std::vector<double>> bases;
    evaluateBasisDerivatives(t, span, bases);
    
    Eigen::Vector3d v(0, 0, 0);
    for (int i = 0; i <= degree_; i++) {
        // 使用1阶导数基函数直接对原控制点加权
        v += control_points_[span - degree_ + i] * bases[1][i];
    }
    return v;
}

Eigen::Vector3d BSpline::getAcceleration(double t) const {
    // 必须有足够的点和阶数
    if (control_points_.size() < 3 || degree_ < 2) return Eigen::Vector3d::Zero();

    t = std::max(0.0, std::min(t, total_time_));

    int span = findSpan(t);
    std::vector<std::vector<double>> bases;
    evaluateBasisDerivatives(t, span, bases);

    Eigen::Vector3d a(0, 0, 0);
    for (int i = 0; i <= degree_; i++) {
        // 使用2阶导数基函数直接对原控制点加权
        a += control_points_[span - degree_ + i] * bases[2][i];
    }
    return a;
}

Eigen::Vector3d BSpline::getJerk(double t) const {
    // 必须有足够的点和阶数（3阶导数需要阶数≥3）
    if (control_points_.size() < 4 || degree_ < 3) return Eigen::Vector3d::Zero();

    t = std::max(0.0, std::min(t, total_time_));

    int span = findSpan(t);
    std::vector<std::vector<double>> bases;
    evaluateBasisDerivatives(t, span, bases);

    Eigen::Vector3d j(0, 0, 0);
    for (int i = 0; i <= degree_; i++) {
        // 使用3阶导数基函数直接对原控制点加权
        j += control_points_[span - degree_ + i] * bases[3][i];
    }
    return j;
}

// 法向量插值保持不变，但使用高效算法
Eigen::Vector3d BSpline::getNormal(double t) const {
    if (normals_.empty()) return Eigen::Vector3d(0,0,1);
    t = std::max(0.0, std::min(t, total_time_));
    if (t >= total_time_) return normals_.back().normalized();

    int span = findSpan(t);
    std::vector<std::vector<double>> bases;
    evaluateBasisDerivatives(t, span, bases);

    Eigen::Vector3d n(0, 0, 0);
    for (int i = 0; i <= degree_; i++) {
        n += normals_[span - degree_ + i] * bases[0][i];
    }
    return n.normalized();
}

int BSpline::findSpan(double t) const {
    int n = control_points_.size() - 1;
    if (t >= knots_[n + 1]) return n; // 特殊处理末端
    
    // 二分查找标准实现
    int low = degree_;
    int high = n + 1;
    int mid = (low + high) / 2;
    
    while (t < knots_[mid] || t >= knots_[mid + 1]) {
        if (t < knots_[mid]) high = mid;
        else low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

// 移除旧的递归 deBoor 函数

} // namespace ship_painter
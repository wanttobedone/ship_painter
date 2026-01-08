#include "ship_painter/bspline.h"
#include <ros/ros.h>
#include <cmath>

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
    
    // 1. 处理闭合：复制首点到末尾
    control_points_ = contour_points;
    normals_ = normals;
    
    if (closed && contour_points.size() > 1) {
        control_points_.push_back(contour_points[0]);
        normals_.push_back(normals[0]);
    }
    
    // 2. 计算轮廓总长度
    double total_length = 0.0;
    for (size_t i = 1; i < control_points_.size(); i++) {
        total_length += (control_points_[i] - control_points_[i-1]).norm();
    }
    
    // 3. 根据速度计算总时间
    total_time_ = total_length / cruise_speed;
    ROS_INFO("B-spline calculation: length=%.2f m, speed=%.2f m/s, time=%.2f s",
         total_length, cruise_speed, total_time_); 
    
    // 4. 生成弦长参数化节点向量
    generateChordLengthKnots(control_points_, total_time_);
    
    ROS_INFO("B-spline fitted: %zu control points, %.2f seconds, %.2f meters",
             control_points_.size(), total_time_, total_length);
}

void BSpline::generateChordLengthKnots(
    const std::vector<Eigen::Vector3d>& points,
    double total_time) {
    
    int n = points.size();  // 控制点数
    int m = n + degree_ + 1;  // 节点向量长度
    
    knots_.clear();
    knots_.resize(m);
    
    // 1. 前 degree_+1 个节点 = 0 (保证起点插值)
    for (int i = 0; i <= degree_; i++) {
        knots_[i] = 0.0;
    }
    
    // 2. 后 degree_+1 个节点 = total_time (保证终点插值)
    for (int i = m - degree_ - 1; i < m; i++) {
        knots_[i] = total_time;
    }
    
    // 3. 中间节点：弦长参数化
    std::vector<double> cumulative_length(n, 0.0);
    for (int i = 1; i < n; i++) {
        cumulative_length[i] = cumulative_length[i-1] + 
                               (points[i] - points[i-1]).norm();
    }
    
    double total_length = cumulative_length[n-1];
    
    for (int i = degree_ + 1; i < m - degree_ - 1; i++) {
        int idx = i - degree_;
        if (idx < n) {
            knots_[i] = (cumulative_length[idx] / total_length) * total_time;
        }
    }
}

Eigen::Vector3d BSpline::getPosition(double t) const {
    // 限制t在有效范围内
    t = std::max(0.0, std::min(t, total_time_));
    
    Eigen::Vector3d result(0, 0, 0);
    int span = findSpan(t);
    
    // De Boor算法计算B样条值
    for (int i = 0; i <= degree_; i++) {
        int idx = span - degree_ + i;
        if (idx >= 0 && idx < static_cast<int>(control_points_.size())) {
            double basis = deBoor(idx, degree_, t, knots_);
            result += control_points_[idx] * basis;
        }
    }
    
    return result;
}

Eigen::Vector3d BSpline::getVelocity(double t) const {
    // 数值微分（简单实现）
    double dt = 0.001;
    Eigen::Vector3d p1 = getPosition(t);
    Eigen::Vector3d p2 = getPosition(t + dt);
    return (p2 - p1) / dt;
}

Eigen::Vector3d BSpline::getAcceleration(double t) const {
    // 数值二阶微分
    double dt = 0.001;
    Eigen::Vector3d v1 = getVelocity(t);
    Eigen::Vector3d v2 = getVelocity(t + dt);
    return (v2 - v1) / dt;
}

Eigen::Vector3d BSpline::getNormal(double t) const {
    // 1. 边界保护
    if (t <= 0.0) return normals_.front().normalized();
    if (t >= total_time_) return normals_.back().normalized();

    // 2. 核心修复：使用De Boor算法对法向量进行插值
    // 原理与 getPosition 完全一致，保证 P(t) 和 N(t) 严格同步
    int span = findSpan(t);

    // 初始化结果向量
    Eigen::Vector3d result = Eigen::Vector3d::Zero();

    // 遍历基函数支撑区间 (p+1个控制点)
    for (int i = 0; i <= degree_; i++) {
        int idx = span - degree_ + i;
        if (idx >= 0 && idx < static_cast<int>(normals_.size())) {
            // 计算基函数值 N_{i,p}(t)
            double basis = deBoor(idx, degree_, t, knots_);
            result += normals_[idx] * basis;
        }
    }

    // 3. 归一化 (插值后的向量长度可能不为1)
    if (result.norm() > 1e-6) {
        result.normalize();
    } else {
        // 异常回退：如果插值结果为0，返回最近的控制点法向
        int safe_idx = std::min((int)normals_.size()-1, std::max(0, span));
        return normals_[safe_idx].normalized();
    }

    return result;
}

int BSpline::findSpan(double t) const {
    int n = control_points_.size() - 1;
    
    // 特殊情况
    if (t >= knots_[n + 1]) return n;
    if (t <= knots_[degree_]) return degree_;
    
    // 二分查找
    int low = degree_;
    int high = n + 1;
    int mid = (low + high) / 2;
    
    while (t < knots_[mid] || t >= knots_[mid + 1]) {
        if (t < knots_[mid]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    
    return mid;
}

double BSpline::deBoor(int i, int p, double t, 
                       const std::vector<double>& knots) const {
    // B样条基函数（递归定义）
    if (p == 0) {
        return (t >= knots[i] && t < knots[i+1]) ? 1.0 : 0.0;
    }
    
    double denom1 = knots[i+p] - knots[i];
    double denom2 = knots[i+p+1] - knots[i+1];
    
    double c1 = 0.0, c2 = 0.0;
    
    if (denom1 > 1e-6) {
        c1 = (t - knots[i]) / denom1 * deBoor(i, p-1, t, knots);
    }
    
    if (denom2 > 1e-6) {
        c2 = (knots[i+p+1] - t) / denom2 * deBoor(i+1, p-1, t, knots);
    }
    
    return c1 + c2;
}

} // namespace ship_painter
// ```

// ---

// ### **Step 2: 创建ROS消息定义**

// **文件位置**：`msg/Bspline.msg`
// ```
// # B样条轨迹消息（参考EGO-Planner）
// int32 order                    # B样条阶数
// time start_time                # 轨迹开始时间
// float64 duration               # 轨迹持续时间

// float64[] knots                # 节点向量
// geometry_msgs/Point[] pos_pts  # 位置控制点
// geometry_msgs/Vector3[] normals # 对应的法向量
// ```

// **文件位置**：`msg/BsplineLayer.msg`
// ```
// # 包含所有层的B样条轨迹
// Bspline[] layers               # 每层的B样条
// Bspline[] transitions          # 层间过渡轨迹
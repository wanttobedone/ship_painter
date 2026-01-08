#ifndef BSPLINE_H
#define BSPLINE_H

#include <Eigen/Dense>
#include <vector>

namespace ship_painter {

class BSpline {
public:
    BSpline() : degree_(3) {}
    
    // 从轮廓点拟合B样条（弦长参数化）
    void fitFromContour(
        const std::vector<Eigen::Vector3d>& contour_points,
        const std::vector<Eigen::Vector3d>& normals,
        double cruise_speed,
        bool closed = true  // 是否闭合
    );

    //添加以下setter方法
    void setControlPoints(const std::vector<Eigen::Vector3d>& points) {
        control_points_ = points;
    }

    void setKnots(const std::vector<double>& knots) {
        knots_ = knots;
    }
    
    void setNormals(const std::vector<Eigen::Vector3d>& normals) {
        normals_ = normals;
    }
    
    void setDegree(int degree) {
        degree_ = degree;
    }
    
    void setTotalTime(double time) {
        total_time_ = time;
    }
    
    // 查询函数
    Eigen::Vector3d getPosition(double t) const;
    Eigen::Vector3d getVelocity(double t) const;
    Eigen::Vector3d getAcceleration(double t) const;
    Eigen::Vector3d getNormal(double t) const;  // 插值法向量
    
    // 获取轨迹参数
    double getTotalTime() const { return total_time_; }
    int getDegree() const { return degree_; }
    
    // 获取数据（用于发布）
    const std::vector<Eigen::Vector3d>& getControlPoints() const { 
        return control_points_; 
    }
    const std::vector<double>& getKnots() const { return knots_; }
    const std::vector<Eigen::Vector3d>& getNormals() const {
        return normals_;
    }
    
private:
    int degree_;  // B样条阶数（默认3）
    std::vector<Eigen::Vector3d> control_points_;  // 控制点
    std::vector<Eigen::Vector3d> normals_;         // 对应的法向量
    std::vector<double> knots_;                    // 节点向量
    double total_time_;                            // 总时间
    
    // 辅助函数
    void generateChordLengthKnots(
        const std::vector<Eigen::Vector3d>& points,
        double total_time
    );
    
    int findSpan(double t) const;

    void evaluateBasisDerivatives(double t, int span, 
                                  std::vector<std::vector<double>>& bases) const;
};

} // namespace ship_painter

#endif
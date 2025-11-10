#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Dense>
#include <vector>
#include "clipper2/clipper.h"
#include "ship_painter/bspline.h" 

// 喷涂参数宏定义
#define SPRAY_RADIUS 0.125          // 喷雾半径 (m)
#define SPRAY_WIDTH (2*SPRAY_RADIUS) // 喷涂带宽度 (m)
#define SPRAY_DISTANCE 0.5          // 喷头到表面距离 (m)
#define MAX_HEIGHT_DIFF 0.025       // 同层最大高度差 (m)

class PathPlanner {
public:

    //生成B样条轨迹
    struct BSplineLayer {
        ship_painter::BSpline trajectory;      // 该层的B样条
        double z_height;         // 层高
        int layer_index;         // 层索引
    };
    
    // 生成所有层的B样条轨迹
    std::vector<BSplineLayer> generateBSplineLayers(
        const Eigen::Vector3d& model_position,
        const Eigen::Vector3d& model_rotation
    );

    
    // 生成层间过渡轨迹
    ship_painter::BSpline generateTransition(  
        const BSplineLayer& from_layer,
        const BSplineLayer& to_layer,
        double transition_speed
    );

    struct Waypoint {
        Eigen::Vector3d position;      // 位置
        Eigen::Vector3d surface_normal; // 表面法向量
        Eigen::Vector3d approach_dir;   // 接近方向（-normal）
        double yaw;                     // 偏航角
        double pitch;                   // 俯仰角
        double roll;                    // 翻滚角（用于横向移动）
        bool needs_pitch_adjustment;    // 是否需要俯仰调整
        
        Waypoint() : yaw(0), pitch(0), roll(0), 
                    needs_pitch_adjustment(false) {}
    };

    // 路径层结构
    struct PathLayer {
        double z_center;                // 层中心高度
        std::vector<Waypoint> waypoints; // 航点序列
        bool is_closed;                 // 是否闭合路径
        
        PathLayer(double z) : z_center(z), is_closed(true) {}
    };

    //从已有path_layers生成B样条（用于真机）
    std::vector<BSplineLayer> generateBSplineLayersFromPath(
        const std::vector<PathLayer>& path_layers
    );

    // 点云可视化
    pcl::PointCloud<pcl::PointNormal>::Ptr getProcessedCloud() const;
    pcl::PointCloud<pcl::PointNormal>::Ptr getOriginalCloud() const;

    // 获取Alpha Shape轮廓（用于调试）
    std::vector<std::pair<double, std::vector<Eigen::Vector3d>>> getAlphaShapeContours() const {
        return alphashape_contours_;
    }
    //偏移后
    std::vector<std::pair<double, std::vector<Eigen::Vector3d>>> getOffsetContours() const {
    return offset_contours_;
    }

    // 规划器配置
    struct PlannerConfig {
        double spray_radius;         // 喷雾半径
        double spray_distance;       // 喷头距离
        double layer_height;         // 层间距（通常=2*spray_radius）
        double max_height_variation; // 同层最大高度差
        double voxel_resolution;     // 体素分辨率
        double path_smoothing_alpha; // 路径平滑系数
        double corner_radius;        // 转角半径
        size_t min_points_per_layer; // 每层最少点数
        bool adaptive_pitch;         // 自适应俯仰控制
        bool use_roll_for_translation; // 使用翻滚角进行平移 
        // Clipper相关参数
        double clipper_offset_precision; // Clipper精度因子（用于整数转换）
        double min_offset_segment_length; // 最小偏移段长度
        bool use_clipper_offset;        // 是否使用Clipper进行偏移
        //
        double alpha_shape_value;      // Alpha Shape参数
        size_t target_sample_count;    // 目标采样点数
        double model_display_x;        // RViz中模型显示位置
        double model_display_y;
        double model_display_z;

        double normal_angle_threshold;   // 法向量角度阈值（弧度）
        bool enable_normal_smoothing;    // 是否启用法向量平滑

        double flight_speed;        //速度参数
        double transition_speed;    //层间过渡速度
        
        double resample_spacing;  // 等距插值间隔 (m)
        PlannerConfig() :
            spray_radius(SPRAY_RADIUS),
            spray_distance(SPRAY_DISTANCE),
            layer_height(SPRAY_WIDTH),
            max_height_variation(MAX_HEIGHT_DIFF),
            voxel_resolution(0.02),
            path_smoothing_alpha(0.3),
            corner_radius(0.2),
            min_points_per_layer(10),
            adaptive_pitch(true),
            use_roll_for_translation(true),
            clipper_offset_precision(1.0), // 1m单位精度
            min_offset_segment_length(0.01),   // 1cm
            use_clipper_offset(true),
            normal_angle_threshold(0.26),  // 法向量平滑插值阈值，弧度制
            enable_normal_smoothing(true),
            flight_speed(0.5),//B样条速度字段
            transition_speed(0.3),
            resample_spacing(0.02) {}//步长，默认0.02m
    };

    PathPlanner(const PlannerConfig& config = PlannerConfig());
    
    // 主要接口
    bool loadSTLModel(const std::string& filename);
    std::vector<PathLayer> generateSprayPath(
        const Eigen::Vector3d& model_position,
        const Eigen::Vector3d& model_rotation = Eigen::Vector3d::Zero()
    );
    
    // 获取模型边界
    void getModelBounds(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) const;
    
private:
    PlannerConfig config_;
    pcl::PointCloud<pcl::PointNormal>::Ptr original_cloud_;
    pcl::PointCloud<pcl::PointNormal>::Ptr processed_cloud_;
    bool model_loaded_;
    Eigen::Vector3d model_center_;  // 模型中心点
    
    // 用于调试的轮廓存储
    std::vector<std::pair<double, std::vector<Eigen::Vector3d>>> alphashape_contours_;
    std::vector<std::pair<double, std::vector<Eigen::Vector3d>>> offset_contours_;        // 偏移后轮廓（蓝色）
    
    // 点云处理
    pcl::PointCloud<pcl::PointNormal>::Ptr preprocessPointCloud(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr sampleMeshSurface(
        const pcl::PolygonMesh& mesh, size_t num_samples);
    
    void estimateSurfaceNormals(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
    void orientNormalsConsistently(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
    
    // 层生成
    std::vector<double> computeLayerHeights(double z_min, double z_max);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr extractLayerPoints(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
        double z_center, double tolerance);
    
    // 路径生成
    std::vector<Waypoint> generateLayerPath(pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud, double z_center);
    
    // 轮廓提取alpha shape方法
    std::vector<Eigen::Vector3d> extractAlphaShapeContour(
        pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud,
        double z_center,
        double alpha);
    
    // === Clipper轮廓偏移方法 ===
    std::vector<Eigen::Vector3d> offsetContourWithClipper(
        const std::vector<Eigen::Vector3d>& original_contour,
        double offset_distance,
        double z_center);
    
    // Clipper辅助函数
    Clipper2Lib::PathsD convertToClipperPaths(
        const std::vector<Eigen::Vector3d>& contour);
    
    std::vector<Eigen::Vector3d> convertFromClipperPaths(
        const Clipper2Lib::PathsD& clipper_paths,
        double z_center);
    
    // 3D法向量重建（用于偏移后的轮廓）
    std::vector<Eigen::Vector3d> recomputeNormalsForOffsetContour(
        const std::vector<Eigen::Vector3d>& offset_contour,
        const std::vector<Eigen::Vector3d>& original_contour,
        pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud);
    
 
    
    // 轮廓验证
    bool isValidContour(const std::vector<Eigen::Vector3d>& contour);
   
    // 轮廓处理
    std::vector<Waypoint> createWaypointsFromContour(
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector3d>& normals,
        double z_center);
    
    
    // 路径优化
    std::vector<Eigen::Vector3d> smoothContourAdaptive(
        const std::vector<Eigen::Vector3d>& contour,
        int iterations);
    
    std::vector<Waypoint> optimizeWaypointDistribution(
        const std::vector<Waypoint>& waypoints);
    
    void computeOrientations(std::vector<Waypoint>& waypoints);
    
    // 姿态计算
    void calculateSprayOrientation(Waypoint& waypoint, const Waypoint* prev = nullptr);
    double calculateRollForTranslation(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                                       double current_yaw, double current_pitch);
    
    // 辅助函数
    void transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                            const Eigen::Vector3d& translation,
                            const Eigen::Vector3d& rotation);
    
    bool segmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                          const Eigen::Vector2d& p3, const Eigen::Vector2d& p4);

    
    Eigen::Vector3d computeAverageNormal(const std::vector<Eigen::Vector3d>& normals);
    
    // 辅助函数
    Eigen::Vector3d computeContourCenter(const std::vector<Eigen::Vector3d>& contour);
};

#endif // PATH_PLANNER_H
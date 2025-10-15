#include "ship_painter/path_planner.h"
#include <ros/ros.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <random>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <map>
#include <set>
#include <limits>

PathPlanner::PathPlanner(const PlannerConfig& config) 
    : config_(config), model_loaded_(false) {
    original_cloud_.reset(new pcl::PointCloud<pcl::PointNormal>);
    processed_cloud_.reset(new pcl::PointCloud<pcl::PointNormal>);
    alphashape_contours_.clear();

    config_.use_clipper_offset = true;
    
    ROS_INFO("PathPlanner initialized with Clipper support");
    ROS_INFO(" - Clipper enabled: %s", config_.use_clipper_offset ? "YES" : "NO");
    ROS_INFO(" - Spray distance: %.3f m", config_.spray_distance);
    ROS_INFO(" - Clipper precision: %.1f", config_.clipper_offset_precision);
    ROS_INFO(" - Min segment length: %.3f m", config_.min_offset_segment_length);
}

bool PathPlanner::loadSTLModel(const std::string& filename) {
    ROS_INFO("Loading STL model: %s", filename.c_str());
    
    if (!boost::filesystem::exists(filename)) {
        ROS_ERROR("STL file does not exist: %s", filename.c_str());
        return false;
    }
    
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(filename, mesh) == 0) {
        ROS_ERROR("Failed to load STL file");
        return false;
    }

    size_t target_samples = config_.target_sample_count;
    original_cloud_ = sampleMeshSurface(mesh, target_samples);
    
    if (original_cloud_->empty()) {
        ROS_ERROR("Failed to generate point cloud from STL");
        return false;
    }
    
    ROS_INFO("Generated %zu points from STL", original_cloud_->size());
    
    // estimateSurfaceNormals(original_cloud_);
    // orientNormalsConsistently(original_cloud_);
    
    processed_cloud_ = preprocessPointCloud(original_cloud_);
    model_loaded_ = true;
    
    return true;
}
//预处理后点云可视化
pcl::PointCloud<pcl::PointNormal>::Ptr PathPlanner::getProcessedCloud() const {
    return processed_cloud_;
}
//原始点云可视化
pcl::PointCloud<pcl::PointNormal>::Ptr PathPlanner::getOriginalCloud() const {
    return original_cloud_;
}

std::vector<PathPlanner::PathLayer> PathPlanner::generateSprayPath(const Eigen::Vector3d& model_position, const Eigen::Vector3d& model_rotation) {
    
    std::vector<PathLayer> layers;
    
    if (!model_loaded_) {
        ROS_ERROR("No model loaded!");
        return layers;
    }
    
    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointNormal>);
    *transformed_cloud = *processed_cloud_;
    
    pcl::PointNormal min_pt, max_pt;
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    Eigen::Vector3d stl_center((min_pt.x + max_pt.x) / 2.0,
                               (min_pt.y + max_pt.y) / 2.0,
                               min_pt.z);
    
    Eigen::Vector3d translation = model_position - stl_center;
    transformPointCloud(transformed_cloud, translation, model_rotation);
    
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    ROS_INFO("Model bounds after transform: X[%.3f, %.3f], Y[%.3f, %.3f], Z[%.3f, %.3f]",
             min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
    
    std::vector<double> layer_heights = computeLayerHeights(min_pt.z, max_pt.z);
    
    ROS_INFO("Planning %zu spray layers", layer_heights.size());
    
    for (size_t i = 0; i < layer_heights.size(); ++i) {
        double z_center = layer_heights[i];
        
        pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud = 
            extractLayerPoints(transformed_cloud, z_center, 0.1);//同层高度的点，容忍度为0.1
        
        if (layer_cloud->size() < config_.min_points_per_layer) {
            ROS_WARN("Layer %zu at z=%.3f has too few points (%zu), skipping",
                     i, z_center, layer_cloud->size());
            continue;
        }
        
        std::vector<Waypoint> waypoints = generateLayerPath(layer_cloud, z_center);
        
        if (waypoints.size() >= 3) {
            PathLayer layer(z_center);
            layer.waypoints = waypoints;
            layer.is_closed = true;
            layers.push_back(layer);
            
            ROS_INFO("Generated layer %zu at z=%.3f with %zu waypoints",
                     layers.size(), z_center, waypoints.size());
        }
    }
    
    ROS_INFO("Successfully generated %zu spray layers", layers.size());
    return layers;
}

// 喷涂层生成generateLayerPath
std::vector<PathPlanner::Waypoint> PathPlanner::generateLayerPath(pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud, double z_center) {

    estimateSurfaceNormals(layer_cloud);
    orientNormalsConsistently(layer_cloud);
    
    std::vector<Waypoint> waypoints;
    
    if (layer_cloud->empty()) return waypoints;

    //X向调试信息
    if (!layer_cloud->empty()) {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        
        for (const auto& pt : layer_cloud->points) {
            min_x = std::min(min_x, (double)pt.x);
            max_x = std::max(max_x, (double)pt.x);
            min_y = std::min(min_y, (double)pt.y);
            max_y = std::max(max_y, (double)pt.y);
        }
        
        ROS_INFO("=== Layer z=%.3f Debug Info ===", z_center);
        ROS_INFO("  Total points in layer: %zu", layer_cloud->size());
        ROS_INFO("  X range: [%.3f, %.3f] (width: %.3f)", min_x, max_x, max_x - min_x);
        ROS_INFO("  Y range: [%.3f, %.3f] (height: %.3f)", min_y, max_y, max_y - min_y);
    }
    
    ROS_INFO("=== CLIPPER PATH GENERATION START ===");
    ROS_INFO("Generating path for layer with %zu points at z=%.3f", 
             layer_cloud->size(), z_center);
    
    // 1. 使用Alpha Shape提取轮廓
    double alpha = config_.alpha_shape_value;
    std::vector<Eigen::Vector3d> original_contour;
    original_contour = extractAlphaShapeContour(layer_cloud, z_center, alpha);

    if (original_contour.size() < 3) {
        ROS_ERROR("Insufficient contour points (%zu), skipping layer", original_contour.size());
        return waypoints;
    }
    
    ROS_INFO(" Extracted %zu original contour points", original_contour.size());
    
    // 分析原始轮廓的尺寸
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    
    for (const auto& pt : original_contour) {
        min_x = std::min(min_x, pt.x());
        max_x = std::max(max_x, pt.x());
        min_y = std::min(min_y, pt.y());
        max_y = std::max(max_y, pt.y());
    }
    
    double contour_width = max_x - min_x;
    double contour_height = max_y - min_y;
    
    ROS_INFO("Original contour dimensions: %.3f x %.3f", contour_width, contour_height);
    ROS_INFO("Center: (%.3f, %.3f)", (min_x + max_x) / 2, (min_y + max_y) / 2);

    // 2. 使用Clipper进行轮廓偏移，但使用渐进式偏移
    ROS_INFO("Starting progressive Clipper offsetting...");
    
    std::vector<Eigen::Vector3d> spray_positions;
    
    // 如果偏移距离很大，尝试分步偏移
    double target_offset = config_.spray_distance;
    if (target_offset > std::min(contour_width, contour_height) * 0.3) {
        ROS_WARN("⚠️ Large offset detected, using progressive offsetting");
        
        // 分步偏移：每步最多偏移轮廓尺寸的20%
        double max_step = std::min(contour_width, contour_height) * 0.2;
        std::vector<Eigen::Vector3d> current_contour = original_contour;
        double remaining_offset = target_offset;
        
        while (remaining_offset > 1e-6 && !current_contour.empty()) {
            double step_offset = std::min(remaining_offset, max_step);
            
            ROS_INFO("   Progressive step: %.3f m (remaining: %.3f m)", 
                     step_offset, remaining_offset);
            
            std::vector<Eigen::Vector3d> step_result = offsetContourWithClipper(
                current_contour, step_offset, z_center);
            
            if (step_result.empty()) {
                ROS_WARN("   Step failed, stopping progressive offset");
                break;
            }
            
            current_contour = step_result;
            remaining_offset -= step_offset;
        }
        
        spray_positions = current_contour;
        
    } else {
        // 一步偏移
        spray_positions = offsetContourWithClipper(
            original_contour, target_offset, z_center);
    }
    
    if (spray_positions.size() < 3) {
        ROS_ERROR("CLIPPER OFFSET FAILED! Generated only %zu points", spray_positions.size());
        ROS_ERROR("Original contour: %zu points, %.3f x %.3f", 
                  original_contour.size(), contour_width, contour_height);
        ROS_ERROR("Target offset: %.3f m", target_offset);
        return waypoints;
    }
    
    ROS_INFO("Clipper offset successful: %zu -> %zu points", 
             original_contour.size(), spray_positions.size());
    
    // *** 保存偏移后的轮廓用于调试可视化 ***
    offset_contours_.push_back({z_center, spray_positions});
    
    // 3. 继续后续处理...
    ROS_INFO("Recomputing normals for offset contour...");
    std::vector<Eigen::Vector3d> normals = recomputeNormalsForOffsetContour(
        spray_positions, original_contour, layer_cloud);
    
    // 4. 验证偏移后的轮廓
    if (!isValidContour(spray_positions)) {
        ROS_WARN("Contour has self-intersections at layer z=%.3f, but continuing without fix", z_center);
    }
    
    // 5. 后续处理...
    spray_positions = smoothContourAdaptive(spray_positions, 1);
    waypoints = createWaypointsFromContour(spray_positions, normals, z_center);
    waypoints = optimizeWaypointDistribution(waypoints);
    computeOrientations(waypoints);

    ROS_INFO("=== CLIPPER PATH GENERATION COMPLETE ===");
    ROS_INFO("Final result: %zu waypoints for layer at z=%.3f", waypoints.size(), z_center);
    
    return waypoints;
}

// 使用CGAL的Alpha Shape算法提取精确的轮廓
std::vector<Eigen::Vector3d> PathPlanner::extractAlphaShapeContour(pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud, double z_center, double alpha) 
{
    
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::FT FT;
    typedef K::Point_2 Point_2;
    typedef K::Segment_2 Segment_2;
    
    typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
    typedef CGAL::Alpha_shape_face_base_2<K> Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb,Fb> Tds;
    typedef CGAL::Delaunay_triangulation_2<K,Tds> Triangulation_2;
    typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;
    
    ROS_INFO("CGAL Alpha Shape processing %zu points at z=%.3f with alpha=%.3f", 
             layer_cloud->size(), z_center, alpha);
    
    // 1. 将点云投影到2D平面
    std::vector<Point_2> points_2d;
    std::map<Point_2, size_t> point_to_index;  // 用于追踪原始点
    
    for (size_t i = 0; i < layer_cloud->size(); ++i) {
        const auto& pt = layer_cloud->points[i];
        Point_2 p2d(pt.x, pt.y);
        points_2d.push_back(p2d);
        point_to_index[p2d] = i;
    }
    
    // 2. 构建Alpha Shape
    Alpha_shape_2 alpha_shape(points_2d.begin(), points_2d.end(),
                              FT(alpha * alpha),  // CGAL使用alpha的平方
                              Alpha_shape_2::GENERAL);
    
    // 3. 提取边界边
    std::vector<Segment_2> segments;
    std::set<Point_2> boundary_vertices;
    
    for (auto it = alpha_shape.alpha_shape_edges_begin();
         it != alpha_shape.alpha_shape_edges_end(); ++it) {
        
        auto edge = *it;
        auto face = edge.first;
        int i = edge.second;
        
        // 获取边的两个顶点
        Point_2 p1 = face->vertex((i+1)%3)->point();
        Point_2 p2 = face->vertex((i+2)%3)->point();
        
        // 检查这条边是否在alpha shape的边界上
        if (alpha_shape.classify(edge) == Alpha_shape_2::REGULAR) {
            segments.push_back(Segment_2(p1, p2));
            boundary_vertices.insert(p1);
            boundary_vertices.insert(p2);
        }
    }
    
    // 4. 将边组织成有序轮廓
    std::vector<Eigen::Vector3d> contour;
    
    if (!segments.empty()) {
        // 构建边的连接图
        std::map<Point_2, std::vector<Point_2>> adjacency;
        for (const auto& seg : segments) {
            adjacency[seg.source()].push_back(seg.target());
            adjacency[seg.target()].push_back(seg.source());
        }
        
        // 从任意点开始遍历轮廓
        std::set<Point_2> visited;
        Point_2 current = segments[0].source();
        Point_2 start = current;
        
        do {
            contour.push_back(Eigen::Vector3d(
                CGAL::to_double(current.x()),
                CGAL::to_double(current.y()),
                z_center
            ));
            visited.insert(current);
            
            // 找下一个未访问的相邻点
            bool found_next = false;
            for (const auto& next : adjacency[current]) {
                if (visited.find(next) == visited.end() || 
                    (next == start && visited.size() > 2)) {
                    current = next;
                    found_next = true;
                    break;
                }
            }
            
            if (!found_next) break;
            
        } while (current != start && visited.size() < boundary_vertices.size());
    }
    
    ROS_INFO("CGAL Alpha Shape extracted %zu contour points", contour.size());
    // 保存原始轮廓用于可视化
    alphashape_contours_.push_back({z_center, contour});

    // === 调试信息：显示Alpha Shape提取结果 ===
    if (!contour.empty()) {
        double min_x_contour = std::numeric_limits<double>::max();
        double max_x_contour = std::numeric_limits<double>::lowest();
        
        for (const auto& pt : contour) {
            min_x_contour = std::min(min_x_contour, pt.x());
            max_x_contour = std::max(max_x_contour, pt.x());
        }
        
        ROS_INFO("  Alpha Shape contour: %zu points", contour.size());
        ROS_INFO("  Contour X range: [%.3f, %.3f] (width: %.3f)", 
                 min_x_contour, max_x_contour, max_x_contour - min_x_contour);
        ROS_INFO("===============================");
    }
    
    return contour;
}

// 验证轮廓是否有效（无自交叉）
bool PathPlanner::isValidContour(const std::vector<Eigen::Vector3d>& contour) {
    if (contour.size() < 3) return false;
    
    // 检查是否有自交叉
    for (size_t i = 0; i < contour.size(); ++i) {
        size_t next_i = (i + 1) % contour.size();
        
        for (size_t j = i + 2; j < contour.size(); ++j) {
            if (j == contour.size() - 1 && i == 0) continue;  // 首尾相连是允许的
            
            size_t next_j = (j + 1) % contour.size();
            
            Eigen::Vector2d p1(contour[i].x(), contour[i].y());
            Eigen::Vector2d p2(contour[next_i].x(), contour[next_i].y());
            Eigen::Vector2d p3(contour[j].x(), contour[j].y());
            Eigen::Vector2d p4(contour[next_j].x(), contour[next_j].y());
            
            if (segmentsIntersect(p1, p2, p3, p4)) {
                return false;
            }
        }
    }
    
    return true;
}


// 从轮廓创建航点
std::vector<PathPlanner::Waypoint> PathPlanner::createWaypointsFromContour(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<Eigen::Vector3d>& normals,
    double z_center) {
    
    std::vector<Waypoint> waypoints;
    
    for (size_t i = 0; i < positions.size(); ++i) {
        Waypoint wp;
        wp.position = positions[i];
        // wp.position.z() = z_center;
        
        if (i < normals.size()) {
            wp.surface_normal = normals[i];
        } else {
            wp.surface_normal = Eigen::Vector3d(0, 0, -1);
        }
        
        wp.approach_dir = -wp.surface_normal;
        waypoints.push_back(wp);
    }
    
    // 确保路径闭合
    if (waypoints.size() > 2) {
        Waypoint closing_wp = waypoints[0];
        waypoints.push_back(closing_wp);
    }
    
    return waypoints;
}


//计算层高
std::vector<double> PathPlanner::computeLayerHeights(double z_min, double z_max) {
    std::vector<double> heights;
    
    double z_start = z_max - config_.spray_radius;
    
    for (double z = z_start; z >= z_min + config_.spray_radius; z -= config_.layer_height) {
        heights.push_back(z);
    }
    
    if (heights.empty() || 
        (heights.back() - z_min > config_.spray_radius + 0.01)) {
        if (heights.empty() || 
            heights.back() - (z_min + config_.spray_radius) > config_.layer_height * 0.5) {
            heights.push_back(z_min + config_.spray_radius);
        }
    }
    
    ROS_INFO("Generated %zu layers from z=%.3f to z=%.3f", 
             heights.size(), 
             heights.empty() ? 0.0 : heights.front(),
             heights.empty() ? 0.0 : heights.back());
    
    return heights;
}
//提取轮廓点
pcl::PointCloud<pcl::PointNormal>::Ptr PathPlanner::extractLayerPoints(
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
    double z_center, double tolerance) {
    
    pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud(new pcl::PointCloud<pcl::PointNormal>);
    
    double strict_tolerance = std::min(tolerance, config_.max_height_variation);

    std::vector<pcl::PointNormal> candidate_points;
        std::vector<double> z_distances;
        
        for (const auto& point : cloud->points) {
            double z_diff = std::abs(point.z - z_center);
            candidate_points.push_back(point);
            z_distances.push_back(z_diff);
        }
        
        if (!candidate_points.empty()) {
            // 找到最小Z距离
            double min_z_diff = *std::min_element(z_distances.begin(), z_distances.end());
            
            // 允许一个很小的容差（考虑浮点精度）
            double precision_tolerance = std::max(tolerance, 1e-6);
            
            for (size_t i = 0; i < candidate_points.size(); ++i) {
                if (z_distances[i] <= min_z_diff + precision_tolerance) {
                    layer_cloud->push_back(candidate_points[i]);
                }
            }
        }
    
    // std::vector<pcl::PointNormal> candidate_points;
    for (const auto& point : cloud->points) {
        double z_diff = std::abs(point.z - z_center);
        if (z_diff <= strict_tolerance) {
            candidate_points.push_back(point);
        }
    }
    
    if (!candidate_points.empty()) {
        double avg_z = 0.0;
        for (const auto& pt : candidate_points) {
            avg_z += pt.z;
        }
        avg_z /= candidate_points.size();
        
        for (const auto& pt : candidate_points) {
            if (std::abs(pt.z - avg_z) <= config_.max_height_variation) {
                layer_cloud->push_back(pt);
            }
        }
    }
    
    ROS_DEBUG("Extracted %zu points for layer at z=%.3f (tolerance=%.3f)", 
              layer_cloud->size(), z_center, strict_tolerance);
    
    return layer_cloud;
}
//优化航点
std::vector<PathPlanner::Waypoint> PathPlanner::optimizeWaypointDistribution(
    const std::vector<Waypoint>& waypoints) {
    
    if (waypoints.size() < 3) return waypoints;
    
    std::vector<Waypoint> optimized;
    
    double target_spacing = 0.0002;  // 2cm
    
    optimized.push_back(waypoints[0]);
    double accumulated_dist = 0.0;
    
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dist = (waypoints[i].position - waypoints[i-1].position).norm();
        accumulated_dist += dist;
        
        if (accumulated_dist >= target_spacing) {
            optimized.push_back(waypoints[i]);
            accumulated_dist = 0.0;
        }
    }
    
    if ((optimized.back().position - waypoints.back().position).norm() > 0.1) {
        optimized.push_back(waypoints.back());
    }
    
    return optimized;
}
//平滑路径
std::vector<Eigen::Vector3d> PathPlanner::smoothContourAdaptive(
    const std::vector<Eigen::Vector3d>& contour,
    int iterations) {
    
    if (contour.size() < 3) return contour;
    
    std::vector<Eigen::Vector3d> smoothed = contour;
    double original_z = contour[0].z();  // 记录原始Z值
    
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<Eigen::Vector3d> new_contour = smoothed;
        
        for (size_t i = 0; i < smoothed.size(); ++i) {
            size_t prev = (i == 0) ? smoothed.size() - 1 : i - 1;
            size_t next = (i + 1) % smoothed.size();
            
            Eigen::Vector3d v1 = (smoothed[i] - smoothed[prev]).normalized();
            Eigen::Vector3d v2 = (smoothed[next] - smoothed[i]).normalized();
            double angle = std::acos(std::max(-1.0, std::min(1.0, v1.dot(v2))));
            
            double smooth_factor = (angle > M_PI/4) ? 0.1 : 0.5;
            
            new_contour[i] = (1.0 - smooth_factor) * smoothed[i] + 
                            smooth_factor * 0.5 * (smoothed[prev] + smoothed[next]);
            
            new_contour[i].z() = original_z;  // 修改：确保Z值保持不变
        }
        
        smoothed = new_contour;
    }
    
    return smoothed;
}
//计算朝向
void PathPlanner::computeOrientations(std::vector<Waypoint>& waypoints) {
    if (waypoints.size() < 2) return;
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto& wp = waypoints[i];
        
        Eigen::Vector3d movement_dir;
        if (i < waypoints.size() - 1) {
            movement_dir = waypoints[i+1].position - wp.position;
        } else if (i > 0) {
            movement_dir = wp.position - waypoints[i-1].position;
        } else {
            movement_dir = Eigen::Vector3d(1, 0, 0);
        }
        movement_dir.z() = 0;
        if (movement_dir.norm() > 1e-6) {
            movement_dir.normalize();
        }
        
        calculateSprayOrientation(wp, i > 0 ? &waypoints[i-1] : nullptr);
        
        if (config_.use_roll_for_translation && i > 0) {
            wp.roll = calculateRollForTranslation(
                waypoints[i-1].position, wp.position,
                wp.yaw, wp.pitch
            );
        }
    }
}

void PathPlanner::calculateSprayOrientation(Waypoint& waypoint, const Waypoint* prev) {
    Eigen::Vector3d spray_dir = -waypoint.surface_normal;
    spray_dir.normalize();
    
    waypoint.yaw = std::atan2(spray_dir.y(), spray_dir.x());
    
    double horizontal_dist = std::sqrt(spray_dir.x() * spray_dir.x() + spray_dir.y() * spray_dir.y());
    waypoint.pitch = std::atan2(-spray_dir.z(), horizontal_dist);
    
    waypoint.pitch = std::max(-M_PI/6, std::min(M_PI/6, waypoint.pitch));
}

double PathPlanner::calculateRollForTranslation(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to,
    double current_yaw, double current_pitch) {
    
    Eigen::Vector3d movement = to - from;
    movement.z() = 0;
    
    if (movement.norm() < 1e-6) return 0.0;
    
    Eigen::Matrix3d R_yaw, R_pitch;
    R_yaw = Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ());
    R_pitch = Eigen::AngleAxisd(current_pitch, Eigen::Vector3d::UnitY());
    
    Eigen::Matrix3d R = R_yaw * R_pitch;
    Eigen::Vector3d movement_body = R.transpose() * movement;
    
    double max_roll = M_PI / 12;
    double roll = std::atan2(movement_body.y(), movement.norm()) * 0.5;
    roll = std::max(-max_roll, std::min(max_roll, roll));
    
    return roll;
}

bool PathPlanner::segmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                    const Eigen::Vector2d& p3, const Eigen::Vector2d& p4) {
    
    Eigen::Vector2d d1 = p2 - p1;
    Eigen::Vector2d d2 = p4 - p3;
    Eigen::Vector2d d3 = p3 - p1;
    
    double cross = d1.x() * d2.y() - d1.y() * d2.x();
    
    if (std::abs(cross) < 1e-10) return false;
    
    double t1 = (d3.x() * d2.y() - d3.y() * d2.x()) / cross;
    double t2 = (d3.x() * d1.y() - d3.y() * d1.x()) / cross;
    
    return (t1 > 0 && t1 < 1 && t2 > 0 && t2 < 1);
}

//表面采样为点云，按平均密度法
pcl::PointCloud<pcl::PointNormal>::Ptr PathPlanner::sampleMeshSurface(const pcl::PolygonMesh& mesh, size_t num_samples) {
    
    pcl::PointCloud<pcl::PointNormal>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointNormal>);
    
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);
    
    if (mesh.polygons.empty()) {
        for (const auto& pt : vertices.points) {
            pcl::PointNormal pn;
            pn.x = pt.x; pn.y = pt.y; pn.z = pt.z;
            sampled_cloud->push_back(pn);
        }
        return sampled_cloud;
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    // 第一步：分析所有三角形，计算总面积
    std::vector<double> triangle_areas;
    std::vector<Eigen::Vector3f> triangle_normals;
    std::map<size_t, size_t> polygon_to_triangle_map;
    
    double total_area = 0.0;
    size_t triangle_idx = 0;
    size_t total_polygons = mesh.polygons.size();
    size_t skipped_triangles = 0;
    
    // 分析阶段
    for (size_t polygon_idx = 0; polygon_idx < total_polygons; ++polygon_idx) {
        const auto& polygon = mesh.polygons[polygon_idx];
        
        if (polygon.vertices.size() < 3) {
            skipped_triangles++;
            continue;
        }
        
        Eigen::Vector3f v0 = vertices[polygon.vertices[0]].getVector3fMap();
        Eigen::Vector3f v1 = vertices[polygon.vertices[1]].getVector3fMap();
        Eigen::Vector3f v2 = vertices[polygon.vertices[2]].getVector3fMap();
        
        Eigen::Vector3f normal = (v1 - v0).cross(v2 - v0);
        double area = 0.5 * normal.norm();
        
        if (area <= 1e-6) {
            skipped_triangles++;
            continue;
        }
        
        // 建立映射
        polygon_to_triangle_map[polygon_idx] = triangle_idx;
        
        // 存储数据
        triangle_areas.push_back(area);
        triangle_normals.push_back(normal.normalized());
        total_area += area;
        
        triangle_idx++;
    }
    
    ROS_INFO("=== Uniform Area-Based Sampling ===");
    ROS_INFO("Total polygons: %zu", total_polygons);
    ROS_INFO("Valid triangles: %zu", triangle_idx);
    ROS_INFO("Skipped triangles: %zu", skipped_triangles);
    ROS_INFO("Total surface area: %.6f", total_area);
    
    // 第二步：按面积比例分配采样点
    std::vector<size_t> samples_per_triangle(triangle_areas.size());
    size_t total_assigned_samples = 0;
    
    for (size_t i = 0; i < triangle_areas.size(); ++i) {
        // 基础分配：按面积比例
        double area_ratio = triangle_areas[i] / total_area;
        size_t assigned_samples = static_cast<size_t>(num_samples * area_ratio);
        
        // 确保每个三角形至少有1个样本（如果目标样本数足够）
        if (assigned_samples == 0 && num_samples >= triangle_areas.size()) {
            assigned_samples = 1;
        }
        
        samples_per_triangle[i] = assigned_samples;
        total_assigned_samples += assigned_samples;
    }
    
    // 第三步：处理余数分配
    // 将剩余的样本分配给面积最大的三角形
    if (total_assigned_samples < num_samples) {
        size_t remaining_samples = num_samples - total_assigned_samples;
        
        // 创建面积排序的索引
        std::vector<std::pair<double, size_t>> area_index_pairs;
        for (size_t i = 0; i < triangle_areas.size(); ++i) {
            area_index_pairs.push_back({triangle_areas[i], i});
        }
        std::sort(area_index_pairs.begin(), area_index_pairs.end(), std::greater<>());
        
        // 将余数分配给面积最大的三角形
        for (size_t i = 0; i < remaining_samples && i < area_index_pairs.size(); ++i) {
            size_t triangle_idx = area_index_pairs[i].second;
            samples_per_triangle[triangle_idx]++;
            total_assigned_samples++;
        }
    }
    
    ROS_INFO("Sample allocation completed:");
    ROS_INFO("  Target samples: %zu", num_samples);
    ROS_INFO("  Assigned samples: %zu", total_assigned_samples);
    ROS_INFO("  Sample density: %.2f points/unit_area", (double)total_assigned_samples / total_area);
    
    // 统计分配情况
    size_t min_samples = *std::min_element(samples_per_triangle.begin(), samples_per_triangle.end());
    size_t max_samples = *std::max_element(samples_per_triangle.begin(), samples_per_triangle.end());
    ROS_INFO("  Samples per triangle: min=%zu, max=%zu", min_samples, max_samples);
    
    // 第四步：执行采样
    size_t actual_samples = 0;
    
    for (size_t polygon_idx = 0; polygon_idx < total_polygons; ++polygon_idx) {
        const auto& polygon = mesh.polygons[polygon_idx];
        
        if (polygon.vertices.size() < 3) continue;
        
        // 使用映射查找
        auto it = polygon_to_triangle_map.find(polygon_idx);
        if (it == polygon_to_triangle_map.end()) continue;
        
        size_t triangle_idx = it->second;
        size_t samples_this_triangle = samples_per_triangle[triangle_idx];
        
        if (samples_this_triangle > 0) {
            Eigen::Vector3f v0 = vertices[polygon.vertices[0]].getVector3fMap();
            Eigen::Vector3f v1 = vertices[polygon.vertices[1]].getVector3fMap();
            Eigen::Vector3f v2 = vertices[polygon.vertices[2]].getVector3fMap();
            Eigen::Vector3f normal = triangle_normals[triangle_idx];
            
            for (size_t j = 0; j < samples_this_triangle; ++j) {
                // 重心坐标随机采样
                double r1 = sqrt(dis(gen));
                double r2 = dis(gen);
                
                double a = 1.0 - r1;
                double b = r1 * (1.0 - r2);
                double c = r1 * r2;
                
                Eigen::Vector3f point = a * v0 + b * v1 + c * v2;
                
                pcl::PointNormal pn;
                pn.x = point.x();
                pn.y = point.y();
                pn.z = point.z();
                pn.normal_x = normal.x();
                pn.normal_y = normal.y();
                pn.normal_z = normal.z();
                
                sampled_cloud->push_back(pn);
                actual_samples++;
            }
        }
    }
    
    ROS_INFO("Uniform sampling completed: %zu samples generated", actual_samples);
    
    return sampled_cloud;
}

//采样后计算每个点的法向量
void PathPlanner::estimateSurfaceNormals(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    ne.setSearchMethod(tree);
    ne.setKSearch(30);
    
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    
    for (size_t i = 0; i < cloud->size() && i < normals->size(); ++i) {
        if (pcl::isFinite<pcl::Normal>((*normals)[i])) {
            (*cloud)[i].normal_x = (*normals)[i].normal_x;
            (*cloud)[i].normal_y = (*normals)[i].normal_y;
            (*cloud)[i].normal_z = (*normals)[i].normal_z;
        }
    }
}

void PathPlanner::orientNormalsConsistently(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    if (cloud->empty()) return;
    
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& point : cloud->points) {
        centroid += point.getVector3fMap();
    }
    centroid /= cloud->size();
    
    for (auto& point : cloud->points) {
        Eigen::Vector3f to_centroid = centroid - point.getVector3fMap();
        Eigen::Vector3f normal(point.normal_x, point.normal_y, point.normal_z);
        
        if (normal.dot(to_centroid) > 0) {
            point.normal_x = -point.normal_x;
            point.normal_y = -point.normal_y;
            point.normal_z = -point.normal_z;
        }
    }
}

//点云预处理
pcl::PointCloud<pcl::PointNormal>::Ptr PathPlanner::preprocessPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    
    double voxel_size = 0.001;//体素分辨率1mm
    pcl::VoxelGrid<pcl::PointNormal> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(config_.voxel_resolution, 
                           config_.voxel_resolution, 
                           config_.voxel_resolution);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr downsampled(new pcl::PointCloud<pcl::PointNormal>);
    voxel_grid.filter(*downsampled);
    
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud(downsampled);
    sor.setMeanK(20);
    sor.setStddevMulThresh(2.0);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr denoised(new pcl::PointCloud<pcl::PointNormal>);
    sor.filter(*denoised);
    
    return denoised;
}

//点云转化为航点
void PathPlanner::transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation) {
    
    Eigen::Matrix3d rot_matrix = Eigen::Matrix3d::Identity();
    
    if (rotation.norm() > 1e-6) {
        Eigen::AngleAxisd roll_angle(rotation.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(rotation.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(rotation.z(), Eigen::Vector3d::UnitZ());
        
        rot_matrix = yaw_angle * pitch_angle * roll_angle;
    }
    
    for (auto& point : cloud->points) {
        Eigen::Vector3d pos(point.x, point.y, point.z);
        pos = rot_matrix * pos + translation;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        
        Eigen::Vector3d normal(point.normal_x, point.normal_y, point.normal_z);
        normal = rot_matrix * normal;
        point.normal_x = normal.x();
        point.normal_y = normal.y();
        point.normal_z = normal.z();
    }
}


void PathPlanner::getModelBounds(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) const {
    if (!model_loaded_ || original_cloud_->empty()) {
        min_pt = max_pt = Eigen::Vector3d::Zero();
        return;
    }
    
    pcl::PointNormal pcl_min, pcl_max;
    pcl::getMinMax3D(*original_cloud_, pcl_min, pcl_max);
    
    min_pt = Eigen::Vector3d(pcl_min.x, pcl_min.y, pcl_min.z);
    max_pt = Eigen::Vector3d(pcl_max.x, pcl_max.y, pcl_max.z);
}

// 计算平均法向量
Eigen::Vector3d PathPlanner::computeAverageNormal(const std::vector<Eigen::Vector3d>& normals) {
    if (normals.empty()) return Eigen::Vector3d(0, 0, -1);
    
    Eigen::Vector3d avg_normal(0, 0, 0);
    for (const auto& normal : normals) {
        avg_normal += normal;
    }
    avg_normal.normalize();
    
    return avg_normal;
}

// offsetContourWithClipper函数，偏移提取出的轮廓形状
std::vector<Eigen::Vector3d> PathPlanner::offsetContourWithClipper(const std::vector<Eigen::Vector3d>& original_contour, double offset_distance, double z_center) {
    
    ROS_INFO("lipper offsetContourWithClipper called");
    ROS_INFO("Input: %zu points, offset: %.3fm, z: %.3fm", 
             original_contour.size(), offset_distance, z_center);
    
    if (original_contour.size() < 3) {
        ROS_ERROR("Insufficient points for Clipper offset: %zu", original_contour.size());
        return {};
    }
    
    try {
        // 1. 转换为Clipper路径格式
        ROS_INFO("Converting to Clipper paths...");
        Clipper2Lib::PathsD subject_paths = convertToClipperPaths(original_contour);
        
        if (subject_paths.empty() || subject_paths[0].size() < 3) {
            ROS_ERROR("Failed to create valid Clipper paths");
            return {};
        }
        
        const auto& path = subject_paths[0];
        ROS_INFO("Converted to Clipper format: %zu paths, first path has %zu points", 
                 subject_paths.size(), path.size());
        
        // 2. 检测轮廓方向和尺寸
        double area = Clipper2Lib::Area(path);
        bool is_clockwise = area < 0;  // Clipper2中，负面积表示顺时针
        
        ROS_INFO("Contour analysis:");
        ROS_INFO("- Area: %.3f", std::abs(area));
        ROS_INFO("- Direction: %s", is_clockwise ? "Clockwise" : "Counter-clockwise");
        
        // 计算轮廓的边界盒来估算尺寸
        Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(path);
        double width = bounds.Width();
        double height = bounds.Height();
        double min_dimension = std::min(width, height);
        
        ROS_INFO("- Bounds: %.3f x %.3f", width, height);
        ROS_INFO("- Min dimension: %.3f", min_dimension);
        ROS_INFO("- Offset distance: %.3f", offset_distance);
        
        // 3. 检查偏移距离是否合理
        if (offset_distance >= min_dimension * 0.4) {
            ROS_WARN("Offset distance (%.3f) is large relative to contour size (%.3f)", 
                     offset_distance, min_dimension);
            ROS_WARN("This may cause the contour to disappear completely");
            
            // 自动调整偏移距离
            double safe_offset = min_dimension * 0.3;  // 使用30%的最小尺寸
            ROS_WARN("Adjusting offset distance from %.3f to %.3f", offset_distance, safe_offset);
            offset_distance = safe_offset;
        }
        
        // 4. 确定正确的偏移符号（向外偏移）
        double clipper_offset_value;
        if (is_clockwise) {
            // 顺时针轮廓：负值向外偏移
            clipper_offset_value = -offset_distance * config_.clipper_offset_precision;
        } else {
            // 逆时针轮廓：正值向外偏移
            clipper_offset_value = offset_distance * config_.clipper_offset_precision;
        }
        
        ROS_INFO("Executing Clipper InflatePaths...");
        ROS_INFO("- Raw offset distance: %.3f", offset_distance);
        ROS_INFO("- Precision factor: %.1f", config_.clipper_offset_precision);
        ROS_INFO("- Clipper offset value: %.1f (%s)", 
                 clipper_offset_value, 
                 clipper_offset_value > 0 ? "outward for CCW" : "outward for CW");
        
        // 5. 尝试多种参数组合
        std::vector<std::pair<double, Clipper2Lib::JoinType>> offset_attempts = {
            {clipper_offset_value, Clipper2Lib::JoinType::Round},
            {clipper_offset_value, Clipper2Lib::JoinType::Square},
            {clipper_offset_value * 0.8, Clipper2Lib::JoinType::Round},  // 80%偏移
            {clipper_offset_value * 0.6, Clipper2Lib::JoinType::Round},  // 60%偏移
            {-clipper_offset_value, Clipper2Lib::JoinType::Round},       // 反向尝试
        };
        
        for (size_t attempt = 0; attempt < offset_attempts.size(); ++attempt) {
            double test_offset = offset_attempts[attempt].first;
            Clipper2Lib::JoinType join_type = offset_attempts[attempt].second;
            
            ROS_INFO("Attempt %zu: offset=%.1f, join=%s", 
                     attempt + 1, test_offset,
                     join_type == Clipper2Lib::JoinType::Round ? "Round" : "Square");
            
            Clipper2Lib::PathsD solution_paths = Clipper2Lib::InflatePaths(
                subject_paths,
                test_offset,
                join_type,
                Clipper2Lib::EndType::Polygon,
                2.0  // 弧段精度
            );
            
            if (!solution_paths.empty()) {
                ROS_INFO("Success on attempt %zu!", attempt + 1);
                ROS_INFO("Generated %zu solution path(s)", solution_paths.size());
                
                // 选择最大的路径
                size_t largest_idx = 0;
                size_t largest_size = 0;
                
                for (size_t i = 0; i < solution_paths.size(); ++i) {
                    ROS_INFO("Solution path %zu: %zu points", i, solution_paths[i].size());
                    if (solution_paths[i].size() > largest_size) {
                        largest_size = solution_paths[i].size();
                        largest_idx = i;
                    }
                }
                
                if (largest_size >= 3) {
                    ROS_INFO("Selected path %zu with %zu points", largest_idx, largest_size);
                    
                    Clipper2Lib::PathsD result_paths;
                    result_paths.push_back(solution_paths[largest_idx]);
                    
                    std::vector<Eigen::Vector3d> offset_contour = convertFromClipperPaths(result_paths, z_center);
                    
                    ROS_INFO("Clipper offset completed successfully!");
                    ROS_INFO("Final result: %zu -> %zu points", 
                             original_contour.size(), offset_contour.size());
                    
                    return offset_contour;
                }
            } else {
                ROS_WARN("Attempt %zu failed: empty solution", attempt + 1);
            }
        }
        
        ROS_ERROR("All offset attempts failed!");
        return {};
        
    } catch (const std::exception& e) {
        ROS_ERROR("Clipper offset failed with exception: %s", e.what());
        return {};
    } catch (...) {
        ROS_ERROR("Clipper offset failed with unknown exception");
        return {};
    }
}

// convertToClipperPaths函数，确保轮廓质量
Clipper2Lib::PathsD PathPlanner::convertToClipperPaths(const std::vector<Eigen::Vector3d>& contour) {
    
    Clipper2Lib::PathsD paths;
    Clipper2Lib::PathD path;
    
    if (contour.size() < 3) {
        ROS_ERROR("Contour too small: %zu points", contour.size());
        return paths;
    }
    
    // 1. 转换点并检测重复点
    std::vector<Clipper2Lib::PointD> temp_points;
    const double MIN_DISTANCE = 1e-6;  // 最小距离阈值
    
    for (const auto& pt : contour) {
        Clipper2Lib::PointD clipper_point;
        clipper_point.x = pt.x();
        clipper_point.y = pt.y();
        
        // 检查是否与前一个点太接近
        if (temp_points.empty() || 
            std::sqrt(std::pow(clipper_point.x - temp_points.back().x, 2) + 
                     std::pow(clipper_point.y - temp_points.back().y, 2)) > MIN_DISTANCE) {
            temp_points.push_back(clipper_point);
        }
    }
    
    // 2. 检查首尾点是否重复
    if (temp_points.size() > 2) {
        double dist = std::sqrt(
            std::pow(temp_points.back().x - temp_points.front().x, 2) + 
            std::pow(temp_points.back().y - temp_points.front().y, 2)
        );
        
        if (dist < MIN_DISTANCE) {
            temp_points.pop_back();  // 移除重复的尾点
        }
    }
    
    // 3. 最终检查
    if (temp_points.size() >= 3) {
        path = temp_points;
        paths.push_back(path);
        
        ROS_INFO("Clean contour: %zu -> %zu points", contour.size(), path.size());
    } else {
        ROS_ERROR("Contour cleaning resulted in insufficient points: %zu", temp_points.size());
    }
    
    return paths;
}


// 从Clipper路径格式转换回三维路径
std::vector<Eigen::Vector3d> PathPlanner::convertFromClipperPaths(const Clipper2Lib::PathsD& clipper_paths,double z_center) {
    
    std::vector<Eigen::Vector3d> contour;
    
    if (clipper_paths.empty() || clipper_paths[0].empty()) {
        return contour;
    }
    
    const auto& path = clipper_paths[0];
    contour.reserve(path.size());
    
    // 将Clipper结果转换回3D坐标
    for (const auto& clipper_point : path) {
        Eigen::Vector3d pt;
        pt.x() = clipper_point.x;
        pt.y() = clipper_point.y;
        pt.z() = z_center;
        contour.push_back(pt);
    }
    
    return contour;
}

// 为偏移后的轮廓重新计算法向量
std::vector<Eigen::Vector3d> PathPlanner::recomputeNormalsForOffsetContour(
    const std::vector<Eigen::Vector3d>& offset_contour,
    const std::vector<Eigen::Vector3d>& original_contour,
    pcl::PointCloud<pcl::PointNormal>::Ptr layer_cloud) {
    
    std::vector<Eigen::Vector3d> normals;
    normals.reserve(offset_contour.size());
    
    // 构建KD树用于快速最近邻搜索
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(layer_cloud);
    
    // 为每个偏移后的点计算法向量
    for (size_t i = 0; i < offset_contour.size(); ++i) {
        Eigen::Vector3d normal;
        
        // 方法1: 基于轮廓几何计算法向量
        size_t prev = (i == 0) ? offset_contour.size() - 1 : i - 1;
        size_t next = (i + 1) % offset_contour.size();
        
        Eigen::Vector3d tangent = (offset_contour[next] - offset_contour[prev]).normalized();
        Eigen::Vector3d geometric_normal(-tangent.y(), tangent.x(), 0);
        geometric_normal.normalize();
        
        // 方法2: 从原始点云插值法向量
        pcl::PointNormal query_point;
        query_point.x = offset_contour[i].x();
        query_point.y = offset_contour[i].y();
        query_point.z = offset_contour[i].z();
        
        std::vector<int> indices;
        std::vector<float> distances;
        
        if (kdtree.nearestKSearch(query_point, 5, indices, distances) > 0) {
            Eigen::Vector3d interpolated_normal(0, 0, 0);
            double total_weight = 0;
            
            for (size_t j = 0; j < indices.size(); ++j) {
                const auto& pt = layer_cloud->points[indices[j]];
                double weight = 1.0 / (distances[j] + 1e-6);  // 距离越近权重越大
                
                interpolated_normal += weight * Eigen::Vector3d(pt.normal_x, pt.normal_y, pt.normal_z);
                total_weight += weight;
            }
            
            if (total_weight > 0) {
                interpolated_normal /= total_weight;
                interpolated_normal.normalize();
                
                // 混合几何法向量和插值法向量
                normal = 0.3 * geometric_normal + 0.7 * interpolated_normal;
            } else {
                normal = geometric_normal;
            }
        } else {
            normal = geometric_normal;
        }
        
        // 确保法向量指向外侧
        Eigen::Vector3d center = computeContourCenter(offset_contour);
        Eigen::Vector3d to_center = center - offset_contour[i];
        to_center.z() = 0;  // 只考虑XY平面
        
        if (normal.dot(to_center) > 0) {
            normal = -normal;
        }
        
        normal.normalize();
        normals.push_back(normal);
    }
    
    return normals;
}

// 计算轮廓中心点
Eigen::Vector3d PathPlanner::computeContourCenter(const std::vector<Eigen::Vector3d>& contour) {
    if (contour.empty()) return Eigen::Vector3d::Zero();
    
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& pt : contour) {
        center += pt;
    }
    center /= contour.size();
    return center;
}




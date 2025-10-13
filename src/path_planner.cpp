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

    config_.use_clipper_offset = true;
    
    ROS_INFO("PathPlanner initialized with Clipper support");
    ROS_INFO(" - Clipper enabled: %s", config_.use_clipper_offset ? "YES" : "NO");
    ROS_INFO(" - Spray distance: %.3f m", config_.spray_distance);
    ROS_INFO(" - Clipper precision: %.1f", config_.clipper_offset_precision);
    ROS_INFO(" - Min segment length: %.3f m", config_.min_offset_segment_length);

    //区域生长函数配置参数
    config_.region_growing_config.k_search = 30;
    config_.region_growing_config.smoothness_threshold = 7.0;
    config_.region_growing_config.curvature_threshold = 1.0;
    config_.region_growing_config.min_cluster_size = 200;
    config_.region_growing_config.max_cluster_size = 1000000;
    config_.region_growing_config.normal_radius = 0.03;
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
    // 执行表面分割
    ROS_INFO("Segmenting surfaces using region growing...");
    segmented_regions_ = segmentSurfaces(processed_cloud_, config_.region_growing_config);

    if (segmented_regions_.empty()) {
        ROS_WARN("No regions found after segmentation!");
    } else {
        ROS_INFO("Successfully segmented into %zu regions", segmented_regions_.size());
        
        // 为每个区域分配颜色
        for (size_t i = 0; i < segmented_regions_.size(); ++i) {
            segmented_regions_[i].color_rgb = generateColorForRegion(i, segmented_regions_.size());
            ROS_INFO("  Region %zu: %zu points, Normal=(%.3f,%.3f,%.3f), Color=(%.2f,%.2f,%.2f)",
                    i, segmented_regions_[i].point_count,
                    segmented_regions_[i].average_normal.x(),
                    segmented_regions_[i].average_normal.y(),
                    segmented_regions_[i].average_normal.z(),
                    segmented_regions_[i].color_rgb.x(),
                    segmented_regions_[i].color_rgb.y(),
                    segmented_regions_[i].color_rgb.z());
        }
    }
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

std::vector<PathPlanner::RegionPath> PathPlanner::generateSprayPath(const Eigen::Vector3d& model_position, const Eigen::Vector3d& model_rotation) {
    
    std::vector<RegionPath> regions;
    
    if (!model_loaded_) {
    ROS_ERROR("No model loaded!");
    return regions;
}

    ROS_INFO("=== Generating Spray Path from Segmented Regions ===");

    if (segmented_regions_.empty()) {
        ROS_ERROR("No segmented regions available!");
        return regions;
    }

    // 对每个分割的区域生成路径
    for (size_t i = 0; i < segmented_regions_.size(); ++i) {
        const auto& region_info = segmented_regions_[i];
        
        ROS_INFO("Processing region %zu/%zu: %zu points", 
                i+1, segmented_regions_.size(), region_info.point_count);
        
        // 创建RegionPath对象
        RegionPath region_path;
        region_path.region_id = region_info.region_id;
        region_path.average_normal = region_info.average_normal;
        region_path.centroid = region_info.centroid;
        region_path.point_count = region_info.point_count;
        region_path.color_rgb = region_info.color_rgb;
        region_path.cloud = region_info.cloud;
        
        // 为该区域生成喷涂路径
        region_path.waypoints = generateRegionPath(region_info.cloud, region_info.average_normal);
        
        if (!region_path.waypoints.empty()) {
            regions.push_back(region_path);
            ROS_INFO("  Generated %zu waypoints for region %zu", 
                    region_path.waypoints.size(), i);
        } else {
            ROS_WARN("  No waypoints generated for region %zu", i);
        }
    }

    ROS_INFO("=== Path Generation Complete: %zu regions ===", regions.size());

    return regions;
}

// 按面进行喷涂
std::vector<PathPlanner::Waypoint> PathPlanner::generateRegionPath(pcl::PointCloud<pcl::PointNormal>::Ptr region_cloud, const Eigen::Vector3d& region_normal)
{

    
    std::vector<Waypoint> waypoints;

    ROS_INFO("Generating path for region with %zu points", region_cloud->size());

    // TODO: 实现具体的路径生成逻辑
    // 目前返回空路径，稍后实现
    // 可以基于区域的点云和法向量生成喷涂轨迹

    return waypoints;
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
    
    // double voxel_size = 0.001;//体素分辨率1mm
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

//表面分割函数
std::vector<PathPlanner::RegionInfo> PathPlanner::segmentSurfaces(
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
    const RegionGrowingConfig& config)
{
    std::vector<RegionInfo> regions;
    
    if (!cloud || cloud->empty()) {
        ROS_ERROR("Input cloud is empty for segmentation!");
        return regions;
    }
    
    ROS_INFO("=== Region Growing Segmentation Start ===");
    ROS_INFO("Input cloud size: %zu points", cloud->size());
    
    // 步骤1: 转换为PointXYZ和Normal分离格式（PCL区域生长需要）
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    for (const auto& pt : cloud->points) {
        pcl::PointXYZ xyz;
        xyz.x = pt.x;
        xyz.y = pt.y;
        xyz.z = pt.z;
        xyz_cloud->push_back(xyz);
        
        pcl::Normal normal;
        normal.normal_x = pt.normal_x;
        normal.normal_y = pt.normal_y;
        normal.normal_z = pt.normal_z;
        normals->push_back(normal);
    }
    
    ROS_INFO("Step 1: Converted to XYZ and Normal format");
    
    // 步骤2: 配置区域生长算法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    
    reg.setInputCloud(xyz_cloud);
    reg.setInputNormals(normals);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(config.k_search);
    
    // 转换角度到弧度
    double smoothness_rad = config.smoothness_threshold * M_PI / 180.0;
    reg.setSmoothnessThreshold(smoothness_rad);
    reg.setCurvatureThreshold(config.curvature_threshold);
    
    reg.setMinClusterSize(config.min_cluster_size);
    reg.setMaxClusterSize(config.max_cluster_size);
    
    ROS_INFO("Step 2: Region growing configured");
    ROS_INFO("  - Neighbors: %d", config.k_search);
    ROS_INFO("  - Smoothness: %.1f degrees (%.3f rad)", 
             config.smoothness_threshold, smoothness_rad);
    ROS_INFO("  - Curvature: %.3f", config.curvature_threshold);
    ROS_INFO("  - Min size: %zu, Max size: %zu", 
             config.min_cluster_size, config.max_cluster_size);
    
    // 步骤3: 执行区域生长
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    
    ROS_INFO("Step 3: Found %zu regions", clusters.size());
    
    // 步骤4: 处理每个区域
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& indices = clusters[i].indices;
        
        if (indices.size() < config.min_cluster_size) {
            continue;
        }
        
        RegionInfo region_info;
        region_info.region_id = i;
        region_info.point_count = indices.size();
        
        // 提取区域点云并计算统计信息
        Eigen::Vector3d normal_sum(0, 0, 0);
        Eigen::Vector3d centroid_sum(0, 0, 0);
        
        for (const auto& idx : indices) {
            // 添加点到区域点云
            pcl::PointNormal pn;
            pn.x = cloud->points[idx].x;
            pn.y = cloud->points[idx].y;
            pn.z = cloud->points[idx].z;
            pn.normal_x = cloud->points[idx].normal_x;
            pn.normal_y = cloud->points[idx].normal_y;
            pn.normal_z = cloud->points[idx].normal_z;
            
            region_info.cloud->push_back(pn);
            
            // 累加法向量和位置
            normal_sum += Eigen::Vector3d(pn.normal_x, pn.normal_y, pn.normal_z);
            centroid_sum += Eigen::Vector3d(pn.x, pn.y, pn.z);
        }
        
        // 计算平均值
        region_info.average_normal = normal_sum / indices.size();
        region_info.average_normal.normalize();
        region_info.centroid = centroid_sum / indices.size();
        
        regions.push_back(region_info);
        
        ROS_INFO("Region %zu: Normal=(%.3f, %.3f, %.3f), Centroid=(%.3f, %.3f, %.3f), Points=%zu",
                 i, region_info.average_normal.x(), 
                 region_info.average_normal.y(), 
                 region_info.average_normal.z(),
                 region_info.centroid.x(), 
                 region_info.centroid.y(), 
                 region_info.centroid.z(),
                 region_info.point_count);
    }
    
    ROS_INFO("=== Segmentation Complete: %zu regions extracted ===", regions.size());
    
    return regions;
}

// ========== 颜色生成函数 ==========
Eigen::Vector3d PathPlanner::generateColorForRegion(int region_idx, int total_regions) {
    // HSV色环均匀分布
    double hue = (region_idx * 360.0) / total_regions;
    double saturation = 0.9;
    double value = 0.9;
    
    return hsvToRgb(hue, saturation, value);
}

Eigen::Vector3d PathPlanner::hsvToRgb(double h, double s, double v) {
    double c = v * s;
    double x = c * (1.0 - std::abs(std::fmod(h / 60.0, 2.0) - 1.0));
    double m = v - c;
    
    double r, g, b;
    
    if (h >= 0 && h < 60) {
        r = c; g = x; b = 0;
    } else if (h >= 60 && h < 120) {
        r = x; g = c; b = 0;
    } else if (h >= 120 && h < 180) {
        r = 0; g = c; b = x;
    } else if (h >= 180 && h < 240) {
        r = 0; g = x; b = c;
    } else if (h >= 240 && h < 300) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }
    
    return Eigen::Vector3d(r + m, g + m, b + m);
}


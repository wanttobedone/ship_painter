#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>

/**
 * @brief 扰动特征数据集采集器
 *
 * 将 MDOB 观测结果与飞行状态打包成特征向量，
 * 同时记录扰动地面真值（风力 GT + 机体恒力 GT）作为训练标签，
 * 存储在预分配的内存缓冲区中，在析构时一次性写入 CSV 文件。
 *
 * 控制线程仅做 push_back（纳秒级），不触发任何磁盘 I/O。
 *
 * CSV 表头（23 列 = 1 timestamp + 16 features + 3 wind GT + 3 body force GT）：
 * timestamp, fw_x, fw_y, fw_z, fb_x, fb_y, fb_z,
 * vw_x, vw_y, vw_z, vb_x, vb_y, vb_z, qw, qx, qy, qz,
 * wind_gt_x, wind_gt_y, wind_gt_z, bf_gt_x, bf_gt_y, bf_gt_z
 */
class DatasetCollector {
public:
    /**
     * @param save_path     CSV 文件保存路径
     * @param reserve_size  预分配记录数（默认 500000 条 ≈ 100Hz × 83min）
     */
    explicit DatasetCollector(const std::string& save_path,
                              size_t reserve_size = 500000);

    /** @brief 析构时将缓冲区内容一次性写入 CSV */
    ~DatasetCollector();

    /**
     * @brief 记录一帧数据（仅内存操作）
     *
     * @param timestamp          时间戳 (s)
     * @param f_dist_world       MDOB 世界系合外力残差 (N)
     * @param f_dist_body        MDOB 机体系合外力残差 (N)
     * @param v_world            世界系速度 (m/s)
     * @param v_body             机体系速度 (m/s)
     * @param q_actual           姿态四元数 (w,x,y,z)
     * @param wind_gt_world      世界系风力地面真值 (N)，无风时为零
     * @param body_force_gt_body 机体系恒力地面真值 (N)，未启用时为零
     */
    void recordFrame(double timestamp,
                     const Eigen::Vector3d& f_dist_world,
                     const Eigen::Vector3d& f_dist_body,
                     const Eigen::Vector3d& v_world,
                     const Eigen::Vector3d& v_body,
                     const Eigen::Quaterniond& q_actual,
                     const Eigen::Vector3d& wind_gt_world,
                     const Eigen::Vector3d& body_force_gt_body);

    /** @brief 手动触发落盘（可在轨迹完成时调用） */
    void flush();

    /** @brief 返回当前已记录的帧数 */
    size_t size() const { return buffer_.size(); }

private:
    struct Record {
        double timestamp;
        double fw_x, fw_y, fw_z;
        double fb_x, fb_y, fb_z;
        double vw_x, vw_y, vw_z;
        double vb_x, vb_y, vb_z;
        double qw, qx, qy, qz;
        double wind_gt_x, wind_gt_y, wind_gt_z;
        double bf_gt_x, bf_gt_y, bf_gt_z;
    };

    std::string save_path_;
    std::vector<Record> buffer_;
    bool flushed_;

    void writeCSV();
};

#include "ship_painter/dataset_collector.h"
#include <cstdio>

DatasetCollector::DatasetCollector(const std::string& save_path, size_t reserve_size)
    : save_path_(save_path),
      flushed_(false)
{
    buffer_.reserve(reserve_size);
}

DatasetCollector::~DatasetCollector() {
    if (!flushed_ && !buffer_.empty()) {
        writeCSV();
    }
}

void DatasetCollector::recordFrame(double timestamp,
                                   const Eigen::Vector3d& f_dist_world,
                                   const Eigen::Vector3d& f_dist_body,
                                   const Eigen::Vector3d& v_world,
                                   const Eigen::Vector3d& v_body,
                                   const Eigen::Quaterniond& q_actual,
                                   const Eigen::Vector3d& wind_gt_world,
                                   const Eigen::Vector3d& body_force_gt_body)
{
    Record r;
    r.timestamp = timestamp;
    r.fw_x = f_dist_world.x();
    r.fw_y = f_dist_world.y();
    r.fw_z = f_dist_world.z();
    r.fb_x = f_dist_body.x();
    r.fb_y = f_dist_body.y();
    r.fb_z = f_dist_body.z();
    r.vw_x = v_world.x();
    r.vw_y = v_world.y();
    r.vw_z = v_world.z();
    r.vb_x = v_body.x();
    r.vb_y = v_body.y();
    r.vb_z = v_body.z();
    r.qw = q_actual.w();
    r.qx = q_actual.x();
    r.qy = q_actual.y();
    r.qz = q_actual.z();
    r.wind_gt_x = wind_gt_world.x();
    r.wind_gt_y = wind_gt_world.y();
    r.wind_gt_z = wind_gt_world.z();
    r.bf_gt_x = body_force_gt_body.x();
    r.bf_gt_y = body_force_gt_body.y();
    r.bf_gt_z = body_force_gt_body.z();

    buffer_.push_back(r);
}

void DatasetCollector::flush() {
    if (!flushed_ && !buffer_.empty()) {
        writeCSV();
        flushed_ = true;
    }
}

void DatasetCollector::writeCSV() {
    FILE* fp = std::fopen(save_path_.c_str(), "w");
    if (!fp) {
        return;
    }

    // 设置 64KB 写缓冲区，减少系统调用次数
    char write_buf[65536];
    std::setvbuf(fp, write_buf, _IOFBF, sizeof(write_buf));

    // CSV 表头 (23 列)
    std::fprintf(fp,
        "timestamp,fw_x,fw_y,fw_z,fb_x,fb_y,fb_z,"
        "vw_x,vw_y,vw_z,vb_x,vb_y,vb_z,qw,qx,qy,qz,"
        "wind_gt_x,wind_gt_y,wind_gt_z,bf_gt_x,bf_gt_y,bf_gt_z\n");

    // 批量写入所有记录
    for (const auto& r : buffer_) {
        std::fprintf(fp,
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            r.timestamp,
            r.fw_x, r.fw_y, r.fw_z,
            r.fb_x, r.fb_y, r.fb_z,
            r.vw_x, r.vw_y, r.vw_z,
            r.vb_x, r.vb_y, r.vb_z,
            r.qw, r.qx, r.qy, r.qz,
            r.wind_gt_x, r.wind_gt_y, r.wind_gt_z,
            r.bf_gt_x, r.bf_gt_y, r.bf_gt_z);
    }

    std::fclose(fp);
}

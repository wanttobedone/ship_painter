#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthTestNode {
public:
    DepthTestNode() : nh_("~") {
        // 订阅对齐后的深度图
        depth_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 
                                   1, &DepthTestNode::depthCallback, this);
        ROS_INFO("Depth test node started. Subscribing to aligned depth image...");
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    
    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_image = cv_ptr->image;
            
            // 获取图像中心点坐标
            int center_x = depth_image.cols / 2;
            int center_y = depth_image.rows / 2;
            
            // 读取中心点深度值（单位：毫米）
            uint16_t depth_mm = depth_image.at<uint16_t>(center_y, center_x);
            float depth_m = depth_mm / 1000.0;  // 转换为米
            
            // 读取中心区域的平均深度（更稳定）
            int region_size = 10;  // 20x20像素区域
            float sum = 0;
            int valid_count = 0;
                        for (int dy = -region_size; dy <= region_size; dy++) {
                for (int dx = -region_size; dx <= region_size; dx++) {
                    int x = center_x + dx;
                    int y = center_y + dy;
                    if (x >= 0 && x < depth_image.cols && y >= 0 && y < depth_image.rows) {
                        uint16_t d = depth_image.at<uint16_t>(y, x);
                        if (d > 0) {  // 忽略无效值
                            sum += d;
                            valid_count++;
                        }
                    }
                }
            }
            
            float avg_depth_m = (valid_count > 0) ? (sum / valid_count / 1000.0) : 0.0;
            
            // 打印结果
            ROS_INFO("Center point: %.3f m | Region average: %.3f m | Valid pixels: %d/%d",
                     depth_m, avg_depth_m, valid_count, (2*region_size+1)*(2*region_size+1));
                     
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_test_node");
    DepthTestNode node;
    ros::spin();
    return 0;
}
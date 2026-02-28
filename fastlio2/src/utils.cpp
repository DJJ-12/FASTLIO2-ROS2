#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    int total = 0, passed = 0; // 新增调试代码0227
    for (int i = 0; i < point_num; i += filter_num)
    {
        total++; // 新增调试代码0227
        // Mid-360 去掉 line 限制，只保留 tag 过滤
        // tag & 0x30 == 0x00 表示最高可信度点
        if ((msg->points[i].tag & 0x30) == 0x00)
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
            
            passed++; // 新增调试代码0227
        }
    }
    // 临时调试打印0227
    static int dbg_count = 0;
    if (++dbg_count == 1)
    {
        std::map<int,int> line_dist, tag_dist;
        for (int i = 0; i < point_num; i++)
        {
            line_dist[msg->points[i].line]++;
            tag_dist[msg->points[i].tag & 0x30]++;
        }
        std::cout << "=== Line distribution ===" << std::endl;
        for (auto& kv : line_dist)
            std::cout << "  line=" << kv.first << " count=" << kv.second << std::endl;
        std::cout << "=== Tag distribution ===" << std::endl;
        for (auto& kv : tag_dist)
            std::cout << "  tag&0x30=" << kv.first << " count=" << kv.second << std::endl;
    }
    static int print_count = 0;
    if (++print_count % 50 == 0)
        std::cout << "[livox2PCL] total=" << total << " passed=" << passed << std::endl;
    // 调试代码结束0227
     return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}

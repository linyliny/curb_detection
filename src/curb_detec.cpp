#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <vector>
using namespace std;


// 帮助对点云的排序。
// 升序。（针对y大于0的点）
bool comp_up(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y < b.y;
}
// 降序。（针对y小于0的点）
bool comp_down(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y > b.y;
}


class curbDetector
// 此类用于检测curb。输入点云，返回检测到的curb点组成的点云。主执行函数为detector。
{
    public:
    curbDetector(){}

    pcl::PointCloud<pcl::PointXYZ> detector(const std::vector<pcl::PointCloud<pcl::PointXYZ> > input)
    {
        pc_in = input;
        pcl::PointCloud<pcl::PointXYZ> look_test;

        for (int i = 0; i < 10; i++)
        // 对于每一环进行处理、检测。由于之前我们这里取了64线lidar的10线，所以这里循环次数为10.
        {
            pcl::PointCloud<pcl::PointXYZ> pointsInTheRing = pc_in[i]; // 储存此线上的点。
            pcl::PointCloud<pcl::PointXYZ> pc_left; // 储存y大于0的点（左侧点）。
            pcl::PointCloud<pcl::PointXYZ> pc_right; // 储存y小于0的点（右侧点）。

            pcl::PointCloud<pcl::PointXYZ> cleaned_pc_left; //储存经过算法1处理过后的左侧点。
            pcl::PointCloud<pcl::PointXYZ> cleaned_pc_right; //储存经过算法1处理过后的右侧点。

            pcl::PointXYZ point; // 点的一个载体。
            size_t numOfPointsInTheRing = pointsInTheRing.size(); // 此线上的点的数量。

            for (int idx = 0; idx < numOfPointsInTheRing; idx++)
            // 分开左、右侧点，分别储存到对应的点云。
            {
                point = pointsInTheRing[idx];
                if (point.y >= 0)
                {pc_left.push_back(point);}
                else
                {pc_right.push_back(point);}
            }

            // 排序。（按绝对值升序）
            sort(pc_left.begin(), pc_left.end(), comp_up);
            sort(pc_right.begin(), pc_right.end(), comp_down);

            // cleaned_pc_left = piao_rou(pc_left);
            // cleaned_pc_right = piao_rou(pc_right);

            // 滑动检测curb点。（对应算法2）
            slideForGettingPoints(pc_left, true);
            slideForGettingPoints(pc_right, false);

            // look_test += (cleaned_pc_left + cleaned_pc_right);
        }
    return curb_left + curb_right;

        // 注意检测结束执行reset，将此类参数置零。(zanshibuyong)
    }

    pcl::PointCloud<pcl::PointXYZ> piao_rou(const pcl::PointCloud<pcl::PointXYZ> hairs)
    {
        const float xy_treth = 0.4;
        const float z_treth = 0.1;
        const int n_N = 10;

        pcl::PointXYZ p0;
        pcl::PointXYZ p1;
        bool jump_flag = false;

        pcl::PointCloud<pcl::PointXYZ> cleanedHairs;
        pcl::PointCloud<pcl::PointXYZ> mayDropedHair;
        size_t numOfTheHairs = hairs.size();
        // cout << endl;

        for (int i = 1; i < numOfTheHairs; i++)
        {
            p0 = hairs[i-1];
            p1 = hairs[i];
            if (p0.y == p1.y)
            {continue;}
            // cout << p1.y << " ";
            float p_dist;
            float z_high;

            p_dist = sqrt(((p0.x - p1.x) * (p0.x - p1.x)) + ((p0.y - p1.y) * (p0.y - p1.y)));
            z_high = fabs(p0.z - p1.z);

            // cout << p_dist << " " << z_high << endl;

            if ((p_dist <= xy_treth) && (z_high <= z_treth))
            {
                if (jump_flag)
                {
                    mayDropedHair.push_back(p1);
                    if (mayDropedHair.size() >= n_N)
                    {
                        cleanedHairs = cleanedHairs + mayDropedHair;
                        jump_flag = false;
                        mayDropedHair.clear();
                    }
                }
                else
                {
                    cleanedHairs.push_back(p1);
                }
            }
            else
            {
                jump_flag = true;
                mayDropedHair.clear();
                mayDropedHair.push_back(p1);
            }
        }
        return cleanedHairs;
    }

    int slideForGettingPoints(pcl::PointCloud<pcl::PointXYZ> points, bool isLeftLine)
    {
        int w_0 = 10;
        int w_d = 30;
        int i = 0;

        // some important parameters influence the final performance.
        float xy_thresh = 0.1;
        float z_thresh = 0.06;

        int points_num = points.size();

        while((i + w_d) < points_num)
        {
            float z_max = points[i].z;
            float z_min = points[i].z;

            int idx_ = 0;
            float z_dis = 0;

            for (int i_ = 0; i_ < w_d; i_++)
            {
                float dis = fabs(points[i+i_].z - points[i+i_+1].z);
                if (dis > z_dis) {z_dis = dis; idx_ = i+i_;}
                if (points[i+i_].z < z_min){z_min = points[i+i_].z;}
                if (points[i+i_].z > z_max){z_max = points[i+i_].z;}
            }

            if (fabs(z_max - z_min) >= z_thresh)
            {
                for (int i_ = 0; i_ < (w_d - 1); i_++)
                {
                    float p_dist = sqrt(((points[i + i_].y - points[i + 1 + i_].y) * (points[i + i_].y - points[i + 1 + i_].y)) 
                    + ((points[i + i_].x - points[i + 1 + i_].x) *(points[i + i_].x - points[i + 1 + i_].x)));
                    if (p_dist >= xy_thresh)
                    {
                        if (isLeftLine) {curb_left.push_back(points[i_ + i]);return 0;}
                        else {curb_right.push_back(points[i_ + i]);return 0;}
                    }
                }
                if (isLeftLine) {curb_left.push_back(points[idx_]);return 0;}
                else {curb_right.push_back(points[idx_]);return 0;}
            }
            i += w_0;
        }
    }

    private:
    std::vector<pcl::PointCloud<pcl::PointXYZ> > pc_in;
    pcl::PointCloud<pcl::PointXYZ> curb_left;
    pcl::PointCloud<pcl::PointXYZ> curb_right;
};


class rosTransPoints
{
    public:
    rosTransPoints()
    {
        //设置感兴趣区域的参数。前四个参数设置了此感兴趣区域矩形的
        // 范围，而ring_idx_设置了我们取那几环的点云。
        x_range_up = 17;
        x_range_down = 1;
        y_range_up = 13;
        y_range_down = -5;
        z_range_down = -1.5;
        int ring_idx_[10] = {25, 27, 30, 32, 35, 38, 40, 41, 42, 43};
        memcpy(ring_idx, ring_idx_, sizeof(ring_idx_));

        // 定义接受和发布。
        pub = nh.advertise<sensor_msgs::PointCloud2>("/curb_detection_result", 1);
        sub = nh.subscribe("/velodyne_points",3,&rosTransPoints::call_back,this);
    }

    void call_back(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
    {
        // 回调函数。每次获得点云都会自动执行的函数。
        clock_t startTime,endTime;
        startTime = ros::Time::now().toNSec();
        curbDetector cd;

        pcl::fromROSMsg(*input, cloud_in);

        // Here to add the code for processing the pc(pointcloud).
        cloud_cleaned = cleanPoints(cloud_in);
        // 执行检测。
        cloud_out = cd.detector(cloud_cleaned);

        pcl::toROSMsg(cloud_out, cloud_final);
        cloud_final.header.stamp = ros::Time::now();
        cloud_final.header.frame_id = "velodyne";
        pub.publish (cloud_final);

        endTime = ros::Time::now().toNSec();
        cout << "The run time is:" << (double)(endTime - startTime) / 10e6 << "ms" << endl;
    }

    int hiPoints_WhereAreYouFrom(pcl::PointXYZ p)
    {
        // 计算点所属于的环数。计算过程经过简化，
        // 具体原理可参考此代码：https://github.com/luhongquan66/loam_velodyne/blob/master/src/lib/MultiScanRegistration.cpp
        double angle;
        int scanID;
        angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
        // double test = sqrt(p.x * p.x + p.y * p.y);
        // cout << test << endl;
        scanID = (int)(angle * 134.18714161056457 + 58.81598513011153);

        return scanID;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ> > cleanPoints(pcl::PointCloud<pcl::PointXYZ> pc)
    {
        // 函数作用：
        // 1. 去除Nan点
        // 2. 根据设定的感兴趣区域取点。（包括矩形区域和环数。）
        // 3. 将取得的不同环数的点分别储存，返还。
        size_t cloudSize = pc.size();
        // size_t ringSize;
        pcl::PointXYZ point;
        int scanID_;
        // pcl::PointCloud<pcl::PointXYZ> _laserCloud;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > laserCloudScans(10);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = pc[i].x;
            point.y = pc[i].y;
            point.z = pc[i].z;
            // cout << point << endl;

            if (!pcl_isfinite(point.x) || 
            !pcl_isfinite(point.y) || 
            !pcl_isfinite(point.z))
            {continue;}
            if ((point.x < x_range_down) || (point.x > x_range_up) || (point.y < y_range_down) || (point.y > y_range_up) || (point.z > z_range_down))
            {continue;}

            scanID_ = hiPoints_WhereAreYouFrom(point);
            // cout << scanID_ << endl;
            for (int ring_num = 0;ring_num < 10; ring_num++)
            {
                if (scanID_ == ring_idx[ring_num])
                {laserCloudScans[ring_num].push_back(point);}
            }
        }

        // for (int i = 0; i < 10; i++)
        // {
        //     // _laserCloud += laserCloudScans[i];
        //     ringSize = laserCloudScans[i].size();
        //     if (ringSize > maxpc)
        //     {
        //         maxpc = ringSize;
        //         cout << maxpc << endl;
        //     }
        // }
        return laserCloudScans;
    }

    private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_cleaned;
    sensor_msgs::PointCloud2 cloud_final;

    // parameters help to select the detection scope.
    int x_range_up;
    int x_range_down;
    int y_range_up;
    int y_range_down;
    float z_range_down;
    int ring_idx[10];

    // size_t maxpc = 0;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "curb_detector");
    rosTransPoints start_detec;
    ros::spin();
}

// copyright：neu_auto_vehicle_lab_lxh
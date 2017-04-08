// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "../include/laserRobustOdometry.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_sharp", 2, laserCloudSharpHandler);

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_less_sharp", 2, laserCloudLessSharpHandler);

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_flat", 2, laserCloudFlatHandler);

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>
            ("/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_2", 2, laserCloudFullResHandler);

    ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2>
            ("/imu_trans", 5, imuTransHandler);

    ros::Subscriber subResetSignal = nh.subscribe<sensor_msgs::PointCloud2>
            ("/reset_signal", 2, resetSignalHandler);

    ros::Subscriber subSpeed = nh.subscribe<sensor_msgs::NavSatFix> ("/speed_raw", 50, speedHandler);

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_corner_last", 2);

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>
            ("/laser_cloud_surf_last", 2);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_3", 2);

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id = "/laser_odom";

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;
    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom";

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

    bool isDegenerate = false;
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    std::ofstream logFileOut;
    logFileOut.open("/home/yiluo/LIDAR/workspace/Debug/log/log.txt");

    // Define ICP function
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //                pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
    //                pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    int frameCount = skipFrameNum;
    ros::Rate rate(100);
    bool status = ros::ok();

    while (status)
    {
        ros::spinOnce();

        if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat &&
                newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans &&
                fabs(timeCornerPointsSharp - timeSurfPointsLessFlat) < 0.005 &&
                fabs(timeCornerPointsLessSharp - timeSurfPointsLessFlat) < 0.005 &&
                fabs(timeSurfPointsFlat - timeSurfPointsLessFlat) < 0.005 &&
                fabs(timeLaserCloudFullRes - timeSurfPointsLessFlat) < 0.005 &&
                fabs(timeImuTrans - timeSurfPointsLessFlat) < 0.005)
        {
            newCornerPointsSharp = false;
            newCornerPointsLessSharp = false;
            newSurfPointsFlat = false;
            newSurfPointsLessFlat = false;
            newLaserCloudFullRes = false;
            newImuTrans = false;

            if (!systemInited)
            {
                pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
                cornerPointsLessSharp = laserCloudCornerLast;
                laserCloudCornerLast = laserCloudTemp;

                laserCloudTemp = surfPointsLessFlat;
                surfPointsLessFlat = laserCloudSurfLast;
                laserCloudSurfLast = laserCloudTemp;

                kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
                kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                transformSum[0] += imuPitchStart;
                transformSum[2] += imuRollStart;

                systemInited = true;
                continue;
            }

            transform[3] -= imuVeloFromStartX * scanPeriod;
            transform[4] -= imuVeloFromStartY * scanPeriod;
            transform[5] -= imuVeloFromStartZ * scanPeriod;



            if (false)
            {
                std::cout << "Transform before Optimization: " << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << "\n";
            }

            bool isConverged = false;
            if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
            {
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                useMotionPrior();

                for (int iterCount = 0; iterCount < 25; iterCount++)
                {
                    laserCloudOri->clear();
                    coeffSel->clear();

                    if (iterCount == 0)
                    {
                        cornerPointsSharpTransformed->clear();
                        surfPointsFlatTransformed->clear();
                    }

                    for (int i = 0; i < cornerPointsSharpNum; i++)
                    {
                        TransformToStart(&cornerPointsSharp->points[i], &pointSel);

                        if (iterCount == 0)
                        {
                            cornerPointsSharpTransformed->push_back(pointSel);
                        }

                        if (iterCount % 5 == 0)
                        {
                            std::vector<int> indices;
                            pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
                            kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                            int closestPointInd = -1, minPointInd2 = -1;
                            if (pointSearchSqDis[0] < 25)
                            {
                                closestPointInd = pointSearchInd[0];
                                int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

                                float pointSqDis, minPointSqDis2 = 25;
                                for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
                                {
                                    if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5)
                                    {
                                        break;
                                    }

                                    pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                            (laserCloudCornerLast->points[j].x - pointSel.x) +
                                            (laserCloudCornerLast->points[j].y - pointSel.y) *
                                            (laserCloudCornerLast->points[j].y - pointSel.y) +
                                            (laserCloudCornerLast->points[j].z - pointSel.z) *
                                            (laserCloudCornerLast->points[j].z - pointSel.z);

                                    if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan)
                                    {
                                        if (pointSqDis < minPointSqDis2)
                                        {
                                            minPointSqDis2 = pointSqDis;
                                            minPointInd2 = j;
                                        }
                                    }
                                }
                                for (int j = closestPointInd - 1; j >= 0; j--)
                                {
                                    if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5)
                                    {
                                        break;
                                    }

                                    pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                            (laserCloudCornerLast->points[j].x - pointSel.x) +
                                            (laserCloudCornerLast->points[j].y - pointSel.y) *
                                            (laserCloudCornerLast->points[j].y - pointSel.y) +
                                            (laserCloudCornerLast->points[j].z - pointSel.z) *
                                            (laserCloudCornerLast->points[j].z - pointSel.z);

                                    if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan)
                                    {
                                        if (pointSqDis < minPointSqDis2)
                                        {
                                            minPointSqDis2 = pointSqDis;
                                            minPointInd2 = j;
                                        }
                                    }
                                }
                            }

                            pointSearchCornerInd1[i] = closestPointInd;
                            pointSearchCornerInd2[i] = minPointInd2;
                        }

                        if (pointSearchCornerInd2[i] >= 0)
                        {
                            tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
                            tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

                            float x0 = pointSel.x;
                            float y0 = pointSel.y;
                            float z0 = pointSel.z;
                            float x1 = tripod1.x;
                            float y1 = tripod1.y;
                            float z1 = tripod1.z;
                            float x2 = tripod2.x;
                            float y2 = tripod2.y;
                            float z2 = tripod2.z;

                            float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                    * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                            float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                            float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                            float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                            float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                            float ld2 = a012 / l12;

                            pointProj = pointSel;
                            pointProj.x -= la * ld2;
                            pointProj.y -= lb * ld2;
                            pointProj.z -= lc * ld2;

                            float s = 1;
                            if (iterCount >= 5)
                            {
                                s = 1 - 1.8 * fabs(ld2);

                                //                                // Use Huber norm here
                                //                                if (ld2 > huberWidth)
                                //                                    s = huberWidth / fabs(ld2);
                                //                                else
                                //                                    s = 1.0;
                                //                                s = sqrt(s);
                            }

                            coeff.x = s * la;
                            coeff.y = s * lb;
                            coeff.z = s * lc;
                            coeff.intensity = s * ld2;

                            if (s > 0.1 && ld2 != 0)
                            {
                                laserCloudOri->push_back(cornerPointsSharp->points[i]);
                                coeffSel->push_back(coeff);
                            }

                            //                            if (ld2 != 0)
                            //                            {
                            //                                laserCloudOri->push_back(surfPointsFlat->points[i]);
                            //                                coeffSel->push_back(coeff);
                            //                            }
                        }
                    }

                    for (int i = 0; i < surfPointsFlatNum; i++)
                    {
                        TransformToStart(&surfPointsFlat->points[i], &pointSel);

                        if (iterCount == 0)
                        {
                            surfPointsFlatTransformed->push_back(pointSel);
                        }

                        if (iterCount % 5 == 0)
                        {
                            kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
                            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                            if (pointSearchSqDis[0] < 25)
                            {
                                closestPointInd = pointSearchInd[0];
                                int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

                                float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
                                for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
                                {
                                    if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5)
                                    {
                                        break;
                                    }

                                    pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                            (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                            (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                                    if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan)
                                    {
                                        if (pointSqDis < minPointSqDis2)
                                        {
                                            minPointSqDis2 = pointSqDis;
                                            minPointInd2 = j;
                                        }
                                    } else
                                    {
                                        if (pointSqDis < minPointSqDis3)
                                        {
                                            minPointSqDis3 = pointSqDis;
                                            minPointInd3 = j;
                                        }
                                    }
                                }
                                for (int j = closestPointInd - 1; j >= 0; j--)
                                {
                                    if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5)
                                    {
                                        break;
                                    }

                                    pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                            (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                            (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                                    if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan)
                                    {
                                        if (pointSqDis < minPointSqDis2)
                                        {
                                            minPointSqDis2 = pointSqDis;
                                            minPointInd2 = j;
                                        }
                                    } else
                                    {
                                        if (pointSqDis < minPointSqDis3)
                                        {
                                            minPointSqDis3 = pointSqDis;
                                            minPointInd3 = j;
                                        }
                                    }
                                }
                            }

                            pointSearchSurfInd1[i] = closestPointInd;
                            pointSearchSurfInd2[i] = minPointInd2;
                            pointSearchSurfInd3[i] = minPointInd3;
                        }

                        if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0)
                        {
                            tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
                            tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
                            tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

                            float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                                    - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
                            float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                                    - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
                            float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                                    - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
                            float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

                            float ps = sqrt(pa * pa + pb * pb + pc * pc);
                            pa /= ps;
                            pb /= ps;
                            pc /= ps;
                            pd /= ps;

                            float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                            pointProj = pointSel;
                            pointProj.x -= pa * pd2;
                            pointProj.y -= pb * pd2;
                            pointProj.z -= pc * pd2;

                            float s = 1;
                            if (iterCount >= 5)
                            {
                                s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                                        + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                                //                                // Use Huber norm here
                                //                                if (pd2 > huberWidth)
                                //                                    s = huberWidth / fabs(pd2);
                                //                                else
                                //                                    s = 1.0;
                                //                                s = sqrt(s);
                            }

                            coeff.x = s * pa;
                            coeff.y = s * pb;
                            coeff.z = s * pc;
                            coeff.intensity = s * pd2;

                            if (s > 0.1 && pd2 != 0)
                            {
                                laserCloudOri->push_back(surfPointsFlat->points[i]);
                                coeffSel->push_back(coeff);
                            }

                            //                            if (pd2 != 0)
                            //                            {
                            //                                laserCloudOri->push_back(surfPointsFlat->points[i]);
                            //                                coeffSel->push_back(coeff);
                            //                            }
                        }
                    }

                    int pointSelNum = laserCloudOri->points.size();
                    if (pointSelNum < 10)
                    {
                        std::cout << "Matched Points not Enough!\n";
                        continue;
                    }


                    cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
                    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
                    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
                    for (int i = 0; i < pointSelNum; i++)
                    {
                        pointOri = laserCloudOri->points[i];
                        coeff = coeffSel->points[i];

                        float s = 1;

                        float srx = sin(s * transform[0]);
                        float crx = cos(s * transform[0]);
                        float sry = sin(s * transform[1]);
                        float cry = cos(s * transform[1]);
                        float srz = sin(s * transform[2]);
                        float crz = cos(s * transform[2]);
                        float tx = s * transform[3];
                        float ty = s * transform[4];
                        float tz = s * transform[5];

                        float arx = (-s * crx * sry * srz * pointOri.x + s * crx * crz * sry * pointOri.y + s * srx * sry * pointOri.z
                                + s * tx * crx * sry * srz - s * ty * crx * crz * sry - s * tz * srx * sry) * coeff.x
                                + (s * srx * srz * pointOri.x - s * crz * srx * pointOri.y + s * crx * pointOri.z
                                + s * ty * crz * srx - s * tz * crx - s * tx * srx * srz) * coeff.y
                                + (s * crx * cry * srz * pointOri.x - s * crx * cry * crz * pointOri.y - s * cry * srx * pointOri.z
                                + s * tz * cry * srx + s * ty * crx * cry * crz - s * tx * crx * cry * srz) * coeff.z;

                        float ary = ((-s * crz * sry - s * cry * srx * srz) * pointOri.x
                                + (s * cry * crz * srx - s * sry * srz) * pointOri.y - s * crx * cry * pointOri.z
                                + tx * (s * crz * sry + s * cry * srx * srz) + ty * (s * sry * srz - s * cry * crz * srx)
                                + s * tz * crx * cry) * coeff.x
                                + ((s * cry * crz - s * srx * sry * srz) * pointOri.x
                                + (s * cry * srz + s * crz * srx * sry) * pointOri.y - s * crx * sry * pointOri.z
                                + s * tz * crx * sry - ty * (s * cry * srz + s * crz * srx * sry)
                                - tx * (s * cry * crz - s * srx * sry * srz)) * coeff.z;

                        float arz = ((-s * cry * srz - s * crz * srx * sry) * pointOri.x + (s * cry * crz - s * srx * sry * srz) * pointOri.y
                                + tx * (s * cry * srz + s * crz * srx * sry) - ty * (s * cry * crz - s * srx * sry * srz)) * coeff.x
                                + (-s * crx * crz * pointOri.x - s * crx * srz * pointOri.y
                                + s * ty * crx * srz + s * tx * crx * crz) * coeff.y
                                + ((s * cry * crz * srx - s * sry * srz) * pointOri.x + (s * crz * sry + s * cry * srx * srz) * pointOri.y
                                + tx * (s * sry * srz - s * cry * crz * srx) - ty * (s * crz * sry + s * cry * srx * srz)) * coeff.z;

                        float atx = -s * (cry * crz - srx * sry * srz) * coeff.x + s * crx * srz * coeff.y
                                - s * (crz * sry + cry * srx * srz) * coeff.z;

                        float aty = -s * (cry * srz + crz * srx * sry) * coeff.x - s * crx * crz * coeff.y
                                - s * (sry * srz - cry * crz * srx) * coeff.z;

                        float atz = s * crx * sry * coeff.x - s * srx * coeff.y - s * crx * cry * coeff.z;

                        float d2 = coeff.intensity;

                        matA.at<float>(i, 0) = arx;
                        matA.at<float>(i, 1) = ary;
                        matA.at<float>(i, 2) = arz;
                        matA.at<float>(i, 3) = atx;
                        matA.at<float>(i, 4) = aty;
                        matA.at<float>(i, 5) = atz;
                        matB.at<float>(i, 0) = -0.05 * d2;
                    }
                    cv::transpose(matA, matAt);
                    matAtA = matAt * matA;
                    matAtB = matAt * matB;
                    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

                    if (iterCount == 0)
                    {
                        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                        cv::eigen(matAtA, matE, matV);
                        matV.copyTo(matV2);

                        isDegenerate = false;
                        float eignThre[6] = {10, 10, 10, 10, 10, 10};
                        for (int i = 5; i >= 0; i--)
                        {
                            if (matE.at<float>(0, i) < eignThre[i])
                            {
                                for (int j = 0; j < 6; j++)
                                {
                                    matV2.at<float>(i, j) = 0;
                                }
                                isDegenerate = true;
                            } else
                            {
                                break;
                            }
                        }
                        matP = matV.inv() * matV2;
                    }

                    if (isDegenerate)
                    {
                        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                        matX.copyTo(matX2);
                        matX = matP * matX2;
                    }

                    transform[0] += matX.at<float>(0, 0);
                    transform[1] += matX.at<float>(1, 0);
                    transform[2] += matX.at<float>(2, 0);
                    transform[3] += matX.at<float>(3, 0);
                    transform[4] += matX.at<float>(4, 0);
                    transform[5] += matX.at<float>(5, 0);

                    //                    std::cout << "Incre of this iteration: " << matX.at<float>(0, 0) << " " << matX.at<float>(1, 0) << " " << matX.at<float>(2, 0) << " " << matX.at<float>(3, 0) << " " << matX.at<float>(4, 0) << " " << matX.at<float>(5, 0) << "\n";
                    //                    std::cout << "Transform after one iteration: " << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << "\n";

                    for (int i = 0; i < 6; i++)
                    {
                        if (isnan(transform[i]))
                            transform[i] = 0;
                    }
                    float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(rad2deg(matX.at<float>(2, 0)), 2));
                    float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

                    //                    if (deltaR < 0.1 && deltaT < 0.1)
                    if (deltaR < 0.05 && deltaT < 0.05)
                    {
                        if (false)
                        {
                            std::cout << "LOAM Odometry Optimization Converge in " << iterCount << " iterations!\n";
                        }
                        isConverged = true;
                        break;
                    }
                }
            } else
            {
                std::cout << "Feature Points not Enough!\n";
            }


            // After the optimization
            // Validate the result

            // Clear previous points
            cloud_in->clear();
            cloud_out->clear();
            laserCloudCur->clear();
            laserCloudLast->clear();

            // Last All into target point cloud
            for (int i = 0; i < laserCloudCornerLast->width; i++)
            {
                pcl::PointXYZ point;
                point.x = laserCloudCornerLast->points[i].x;
                point.y = laserCloudCornerLast->points[i].y;
                point.z = laserCloudCornerLast->points[i].z;
                //        point.intensity = laserCloudCornerLast->points[i].intensity;
                if (point.y > groundThreshold)
                    cloud_out->push_back(point);
            }
            for (int i = 0; i < laserCloudSurfLast->width; i++)
            {
                pcl::PointXYZ point;
                point.x = laserCloudSurfLast->points[i].x;
                point.y = laserCloudSurfLast->points[i].y;
                point.z = laserCloudSurfLast->points[i].z;
                //        point.intensity = laserCloudSurfLast->points[i].intensity;
                if (point.y > groundThreshold)
                    cloud_out->push_back(point);

                pcl::PointXYZI pointI;
                pointI.x = laserCloudSurfLast->points[i].x;
                pointI.y = laserCloudSurfLast->points[i].y;
                pointI.z = laserCloudSurfLast->points[i].z;
                pointI.intensity = laserCloudSurfLast->points[i].intensity;
                laserCloudLast->push_back(pointI);
            }
            cloud_out->width = (int) cloud_out->points.size();
            cloud_out->height = 1;
            laserCloudLast->width = (int) laserCloudLast->points.size();
            laserCloudLast->height = 1;

            // Cur All into temp current point cloud (XYZI)
            for (int i = 0; i < surfPointsLessFlat->width; i++)
            {
                pcl::PointXYZI point;
                point.x = surfPointsLessFlat->points[i].x;
                point.y = surfPointsLessFlat->points[i].y;
                point.z = surfPointsLessFlat->points[i].z;
                point.intensity = surfPointsLessFlat->points[i].intensity;

                float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                if (point.y > groundThreshold)
                    laserCloudCur->push_back(point);
            }
            for (int i = 0; i < cornerPointsLessSharp->width; i++)
            {
                pcl::PointXYZI point;
                point.x = cornerPointsLessSharp->points[i].x;
                point.y = cornerPointsLessSharp->points[i].y;
                point.z = cornerPointsLessSharp->points[i].z;
                point.intensity = cornerPointsLessSharp->points[i].intensity;

                float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                if (point.y > groundThreshold)
                    laserCloudCur->push_back(point);
            }
            laserCloudCur->width = (int) laserCloudCur->points.size();
            laserCloudCur->height = 1;

            // Temp current point cloud(XYZI) into current point cloud
            for (int i = 0; i < laserCloudCur->width; i++)
            {
                TransformToStart(&laserCloudCur->points[i], &laserCloudCur->points[i]);

                pcl::PointXYZ point;
                point.x = laserCloudCur->points[i].x;
                point.y = laserCloudCur->points[i].y;
                point.z = laserCloudCur->points[i].z;
                if (point.y > groundThreshold)
                    cloud_in->push_back(point);
            }
            cloud_in->width = (int) cloud_in->points.size();
            cloud_in->height = 1;

            // Evaluate the result
            Eigen::Matrix4f guess = Eigen::Affine3f::Identity().matrix();
            pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> validatePoseEstimate;
            validatePoseEstimate.setMaxRange(0.5);
            float loamScore = validatePoseEstimate.validateTransformation(cloud_in, cloud_out, guess);

            if (false)
            {
                std::cout << "Transform after LOAM Optimization: " << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << "\n";
                std::cout << "LOAM Optimization score: " << loamScore << ", valid range: " << validatePoseEstimate.getMaxRange() << "\n";
            }

            // Do we need Robust ICP?
            bool doRobustICP = false;
            if (isConverged && loamScore > validScoreThreshold) doRobustICP = true;
            //            if (transform[3] < 1.0) doRobustICP = true;
            //            if (!isConverged && loamScore > validScoreDivergedThreshold) doRobustICP = true;
            if (loamScore > validScoreThreshold) doRobustICP = true;
            //            if (fabs(transform[4]) > 0.25 || fabs(transform[5]) > 0.2 || fabs(transform[3]) < 0.80 * speedPrior) doRobustICP = true;


            // Turn to robust ICP for pose estimation
            if (doRobustICP && cloud_in->width > 1000)
            {
                std::cout << "Do Robust ICP! With " << cloud_in->width << " points.\n";
                // Apply motion model
                // Constant velocity model here
                // Revert to transformation from k-1 to k
                transform[0] = transformLast[0];
                transform[1] = transformLast[1];
                transform[2] = transformLast[2];
                transform[3] = transformLast[3];
                transform[4] = transformLast[4];
                transform[5] = transformLast[5];
                transformLOAM[0] = transform[0];
                transformLOAM[1] = transform[1];
                transformLOAM[2] = transform[2];
                transformLOAM[3] = transform[3];
                transformLOAM[4] = transform[4];
                transformLOAM[5] = transform[5];
                cloud_in->clear();

                // Temp current point cloud(XYZI) into current point cloud
                for (int i = 0; i < laserCloudCur->width; i++)
                {
                    TransformToStart(&laserCloudCur->points[i], &laserCloudCur->points[i]);

                    pcl::PointXYZ point;
                    point.x = laserCloudCur->points[i].x;
                    point.y = laserCloudCur->points[i].y;
                    point.z = laserCloudCur->points[i].z;
                    if (point.y > groundThreshold)
                        cloud_in->push_back(point);
                }
                cloud_in->width = (int) cloud_in->points.size();
                cloud_in->height = 1;

                float ICPScore = validatePoseEstimate.validateTransformation(cloud_in, cloud_out, guess);
                if (prinfDebugInfo)
                {
                    std::cout << "Before ICP Optimization score: " << ICPScore << ", valid range: " << validatePoseEstimate.getMaxRange() << "\n";

                    // Do Multiple Iteration Till Converge
                    printf("Calculating...\n");
                }
                double preFitnessScore = DBL_MAX;
                bool converge = false;
                int iter = 0;

                // Set icp parameters
                icp.setRANSACIterations(100);
                icp.setRANSACOutlierRejectionThreshold(2.0);
                // Reject Outlier after Initialization Period
                icp.setMaxCorrespondenceDistance(5.0);
                //                if (frameCount > 10)
                //                    icp.setMaxCorrespondenceDistance(0.5);
                icp.setInputTarget(cloud_out);
                //                std::cout << "RANSACOutlierRejectionThreshold: " << icp.getRANSACOutlierRejectionThreshold() << ", RANSACIterations: " << icp.getRANSACIterations() << "\n";
                pcl::PointCloud<pcl::PointXYZ> Final;

                while (!converge)
                {
                    iter++;
                    if (prinfDebugInfo)
                    {
                        std::cout << "Iter: " << iter << " ";
                    }
                    if (iter >= 3) icp.setMaxCorrespondenceDistance(1.0);

                    Final.clear();
                    icp.setInputSource(cloud_in);
                    //        icp.align(Final, guess);
                    icp.align(Final);
                    if (prinfDebugInfo)
                    {
                        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                                icp.getFitnessScore() << std::endl;
                    }
                    //                    std::cout << "RANSACOutlierRejectionThreshold: " << icp.getRANSACOutlierRejectionThreshold() << ", RANSACIterations: " << icp.getRANSACIterations() << "\n";
                    //                    std::cout << icp.getFinalTransformation() << std::endl;

                    // Check Convergence
                    if (!icp.hasConverged() || (icp.getFitnessScore() > preFitnessScore)) break;
                    // Check Score
                    if (iter > 1)
                        if ((preFitnessScore - icp.getFitnessScore()) < 0.0001) converge = true;

                    // Store prev Score
                    preFitnessScore = icp.getFitnessScore();
                    //        converge = true;

                    Eigen::Matrix4f T_cur_to_prev_mat = icp.getFinalTransformation();
                    Eigen::Affine3f T_cur_to_prev_aff = Eigen::Affine3f::Identity();

                    for (int i = 0; i < 4; i++)
                        for (int j = 0; j < 4; j++)
                            T_cur_to_prev_aff(i, j) = T_cur_to_prev_mat(i, j);
                    //    std::cout << T_cur_to_prev_aff.matrix() << "\n";

                    Eigen::Affine3f T_prev_to_cur_aff = T_cur_to_prev_aff.inverse();
                    //                    std::cout << T_prev_to_cur_aff.matrix() << "\n";

                    float x, y, z, roll, pitch, yaw;
                    pcl::getTranslationAndEulerAngles(T_prev_to_cur_aff, x, y, z, roll, pitch, yaw);
                    //                    std::cout << roll << " " << pitch << " " << yaw << " " << x << " " << y << " " << z << "\n";
                    //    std::cout << -sin(pitch)  << "\n";

                    float rx, ry, rz, tx, ty, tz;
                    GetEulerAnglesAndTranslation(T_prev_to_cur_aff, rx, ry, rz, tx, ty, tz);
                    //                    std::cout << rx << " " << ry << " " << rz << " " << tx << " " << ty << " " << tz << "\n";
                    //    std::cout << -cos(rx) * sin(rz) << "\n";
                    //    std::cout << cos(rx) * cos(rz) << "\n";
                    //    std::cout << sin(ry) * cos(rz) + sin(rx) * cos(ry) * sin(rz) << "\n";
                    //    std::cout << sin(ry) * sin(rz) - sin(rx) * cos(ry) * cos(rz) << "\n";

                    //                    Eigen::Affine3f test_aff = Eigen::Affine3f::Identity();
                    //                    GetTransformationFromEulerAnglesAndTranslation(test_aff, rx, ry, rz, tx, ty, tz);
                    //                    std::cout << std::endl << T_prev_to_cur_aff.matrix() << std::endl;
                    //                    std::cout << std::endl << test_aff.matrix() << std::endl;

                    // Set transform from prev to cur
                    transform[0] += rx;
                    transform[1] += ry;
                    transform[2] += rz;
                    transform[3] += tx;
                    transform[4] += ty;
                    transform[5] += tz;

                    // For all cur points
                    // Transform to the start time frame
                    laserCloudCur->clear();
                    // Cur All into temp cur point cloud
                    for (int i = 0; i < surfPointsLessFlat->width; i++)
                    {
                        pcl::PointXYZI point;
                        TransformToStart(&surfPointsLessFlat->points[i], &point);
                        if (point.y > groundThreshold)
                            laserCloudCur->push_back(point);
                    }
                    for (int i = 0; i < cornerPointsLessSharp->width; i++)
                    {
                        pcl::PointXYZI point;
                        TransformToStart(&cornerPointsLessSharp->points[i], &point);
                        if (point.y > groundThreshold)
                            laserCloudCur->push_back(point);
                    }
                    laserCloudCur->width = (int) laserCloudCur->points.size();
                    laserCloudCur->height = 1;

                    cloud_in->clear();
                    // Temp cur point cloud into cur point cloud
                    for (int i = 0; i < laserCloudCur->width; i++)
                    {
                        pcl::PointXYZ point;
                        point.x = laserCloudCur->points[i].x;
                        point.y = laserCloudCur->points[i].y;
                        point.z = laserCloudCur->points[i].z;

                        cloud_in->push_back(point);
                    }
                    cloud_in->width = (int) cloud_in->points.size();
                    cloud_in->height = 1;
                }

                if (prinfDebugInfo)
                {
                    std::cout << "Transform after ICP Optimization: " << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << "\n";
                }

                // Do LM
                bool doLM = false;
                if (doLM)
                {
                    // Prepare PointCloud
                    laserCloudCur->clear();
                    // Cur All into temp cur point cloud
                    for (int i = 0; i < surfPointsLessFlat->width; i++)
                    {
                        pcl::PointXYZI point;
                        TransformToStart(&surfPointsLessFlat->points[i], &point);
                        laserCloudCur->push_back(point);
                    }
                    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfCurUndistorted(new pcl::PointCloud<pcl::PointXYZI>);
                    for (int i = 0; i < laserCloudCur->width; i++)
                    {
                        laserCloudSurfCurUndistorted->push_back(laserCloudCur->points[i]);
                    }
                    laserCloudSurfCurUndistorted->width = (int) laserCloudSurfCurUndistorted->points.size();
                    laserCloudSurfCurUndistorted->height = 1;


                    std::vector<bool> inliers;
                    bool isLMConverge = false;
                    iter = 0;
                    Eigen::Isometry3d fromCurToRef_estimated = Eigen::Isometry3d::Identity();
                    //                fromCurToRef_estimated.translate(Eigen::Vector3d(0.0, 0.0, 0.0));
                    //                std::cout << "Initial:\n" << fromCurToRef_estimated.matrix() << std::endl;
                    //                OdometryTransformToStart(laserCloudSurfCurUndistorted, fromCurToRef_estimated);


                    while (!isLMConverge && iter < 15)
                    {
                        double outlierThreshold = 2.0;
                        //        outlierThreshold -= iter*0.2;
                        //        outlierThreshold = outlierThreshold < 2.0 ? 2.0 : outlierThreshold;
                        Eigen::Isometry3d fromCurToRef_inc = MappingICP(laserCloudSurfLast, laserCloudSurfCurUndistorted, inliers, outlierThreshold);
                        //                        Eigen::Isometry3d fromCurToRef_inc = OdometryICP(laserCloudLast, laserCloudSurfCurUndistorted, inliers, outlierThreshold);
                        OdometryTransformToStart(laserCloudSurfCurUndistorted, fromCurToRef_inc);
                        std::cout << "iter: " << iter++ << std::endl;
                        //                    std::cout << fromCurToRef_inc.matrix() << std::endl;

                        if (fabs(fromCurToRef_inc.translation()[0]) < 0.005 && fabs(fromCurToRef_inc.translation()[1]) < 0.005 && fabs(fromCurToRef_inc.translation()[2]) < 0.005)
                            isLMConverge = true;

                        fromCurToRef_estimated = fromCurToRef_inc*fromCurToRef_estimated;
                        //                    iter++;
                    }
                    std::cout << "Final:\n" << fromCurToRef_estimated.matrix() << std::endl;

                    // Update transformation with LM estimation
                    Eigen::Matrix4d fromCurToRef_mat = fromCurToRef_estimated.matrix();
                    Eigen::Affine3f fromCurToRef_aff = Eigen::Affine3f::Identity();

                    for (int i = 0; i < 4; i++)
                        for (int j = 0; j < 4; j++)
                            fromCurToRef_aff(i, j) = (float) fromCurToRef_mat(i, j);

                    //                float x, y, z, roll, pitch, yaw;
                    //                pcl::getTranslationAndEulerAngles(T_prev_to_cur_aff, x, y, z, roll, pitch, yaw);
                    //                //                    std::cout << roll << " " << pitch << " " << yaw << " " << x << " " << y << " " << z << "\n";
                    //                //    std::cout << -sin(pitch)  << "\n";

                    float rx, ry, rz, tx, ty, tz;
                    GetEulerAnglesAndTranslation(fromCurToRef_aff, rx, ry, rz, tx, ty, tz);

                    if (fabs(ty) < 0.15 && fabs(tz) < 0.15)
                    {
                        transform[0] += rx;
                        transform[1] += ry;
                        transform[2] += rz;
                        transform[3] += tx;
                        transform[4] += ty;
                        transform[5] += tz;
                    }
                    if (prinfDebugInfo)
                    {
                        std::cout << "Transform after LM Optimization: " << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << "\n";
                    }
                }

                //                if (prinfDebugInfo)
                //                {
                //                    std::cout << "After ICP Optimization score: " << validatePoseEstimate.validateTransformation(cloud_in, cloud_out, guess) << ", valid range: " << validatePoseEstimate.getMaxRange() << "\n";
                //                }
            }

            if (false)
            {
                std::cout << "Transform after Odometry Optimization: " << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << "\n";
            }

            checkMotionEstimation();

            // Pose esimation done!!!!
            frameCount++;
            frameCountSinceStart++;

            // Accumulate the pose
            // Why do it in reverse way?
            float rx, ry, rz, tx, ty, tz;
            //            AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
            //                    -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);
            AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                    -transform[0], -transform[1], -transform[2], rx, ry, rz);

            float x1 = cos(rz) * (transform[3] - imuShiftFromStartX)
                    - sin(rz) * (transform[4] - imuShiftFromStartY);
            float y1 = sin(rz) * (transform[3] - imuShiftFromStartX)
                    + cos(rz) * (transform[4] - imuShiftFromStartY);
            //            float z1 = transform[5] * 1.05 - imuShiftFromStartZ;
            float z1 = transform[5] - imuShiftFromStartZ;

            float x2 = x1;
            float y2 = cos(rx) * y1 - sin(rx) * z1;
            float z2 = sin(rx) * y1 + cos(rx) * z1;

            tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
            ty = transformSum[4] - y2;
            tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

            PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart,
                    imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

            transformSum[0] = rx;
            transformSum[1] = ry;
            transformSum[2] = rz;
            transformSum[3] = tx;
            transformSum[4] = ty;
            transformSum[5] = tz;


            double timeNow = ros::Time::now().toSec();
            if (initFrameROSTime < 0.0)
                initFrameROSTime = timeNow;

            if (prinfDebugInfo)
            {
                //                std::cout << "Now: " << timeNow - initFrameROSTime << " transform before Optimization: " << transformLast[0] << " " << transformLast[1] << " " << transformLast[2] << " " << transformLast[3] << " " << transformLast[4] << " " << transformLast[5] << "\n";
                std::cout << "Now: " << timeNow - initFrameROSTime << " transform after Optimization: " << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << "\n";
                //                std::cout << "Now: " << timeNow - initFrameROSTime << " transformSum: " << transformSum[0] << " " << transformSum[1] << " " << transformSum[2] << " " << transformSum[3] << " " << transformSum[4] << " " << transformSum[5] << "\n\n";
            }

            cornerPointsSharpTransformed->width = (int) cornerPointsSharpTransformed->points.size();
            cornerPointsSharpTransformed->height = 1;
            surfPointsFlatTransformed->width = (int) surfPointsFlatTransformed->points.size();
            surfPointsFlatTransformed->height = 1;

            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, -rx, -ry);

            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = -geoQuat.y;
            laserOdometry.pose.pose.orientation.y = -geoQuat.z;
            laserOdometry.pose.pose.orientation.z = geoQuat.x;
            laserOdometry.pose.pose.orientation.w = geoQuat.w;
            laserOdometry.pose.pose.position.x = tx;
            laserOdometry.pose.pose.position.y = ty;
            laserOdometry.pose.pose.position.z = tz;
            pubLaserOdometry.publish(laserOdometry);

            laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
            laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
            tfBroadcaster.sendTransform(laserOdometryTrans);

            int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
            for (int i = 0; i < cornerPointsLessSharpNum; i++)
            {
                TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
            }

            int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
            for (int i = 0; i < surfPointsLessFlatNum; i++)
            {
                TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
            }

            frameCount++;
            if (frameCount >= skipFrameNum + 1)
            {
                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();
            if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
            {
                kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
                kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
            }

            if (frameCount >= skipFrameNum + 1)
            {
                frameCount = 0;

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    logFileOut.close();

    return 0;
}

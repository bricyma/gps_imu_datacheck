/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   SubmapUtils.cpp
 * Author: yiluo
 *
 * Created on December 15, 2016, 3:14 PM
 */

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>
//#include <random>
#include <iostream>
#include <stdint.h>
#include <map>

//#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include "CeresICP.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

int alignTwoTracks()
{
    std::string refPointCloudPathString = "/home/yiluo/LIDAR/workspace/CeresICP/1479884509.94.pcd";
    std::string srcPointCloudPathString = "/home/yiluo/LIDAR/workspace/CeresICP/1479967869.95.pcd";
    std::string refGPSBackboneInfoPathString = "/home/yiluo/LIDAR/workspace/CeresICP/1479884509.94.txt";
    std::string srcGPSBackboneInfoPathString = "/home/yiluo/LIDAR/workspace/CeresICP/1479967869.95.txt";

    pcl::PointCloud<pcl::PointXYZ>::Ptr subMapRefFull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr subMapCurFull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr subMapCurFinal(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr subMapRef(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr subMapCur(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 cloud_blob;

    pcl::io::loadPCDFile(refPointCloudPathString.c_str(), cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *subMapRefFull); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
    subMapRefFull->width = (int) subMapRefFull->points.size();
    subMapRefFull->height = 1;

    pcl::io::loadPCDFile(srcPointCloudPathString.c_str(), cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *subMapCurFull); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
    subMapCurFull->width = (int) subMapCurFull->points.size();
    subMapCurFull->height = 1;

    std::cout << "Reading gps backbone info...\n";
    Eigen::Affine3d fromLidarToGPSRef = Eigen::Affine3d::Identity();
    Eigen::Affine3d fromLidarToGPSCur = Eigen::Affine3d::Identity();

    ifstream data(refGPSBackboneInfoPathString.c_str());
    std::string line;
    if(getline(data, line))
    {
        std::istringstream iss;
        iss.str(line);
        double tmp;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                iss >> tmp;
                fromLidarToGPSRef(i,j) = tmp;
            }
        }

        for (int i = 0; i < 3; ++i)
            fromLidarToGPSRef(3,i) = 0.0;
        fromLidarToGPSRef(3,3) = 1.0;

        std::cout << fromLidarToGPSRef.matrix() << "\n";;
    }
    data.close();

    data.open(srcGPSBackboneInfoPathString.c_str());
    if(getline(data, line))
    {
        std::istringstream iss;
        iss.str(line);
        double tmp;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                iss >> tmp;
                fromLidarToGPSCur(i,j) = tmp;
            }
        }

        for (int i = 0; i < 3; ++i)
            fromLidarToGPSCur(3,i) = 0.0;
        fromLidarToGPSCur(3,3) = 1.0;

        std::cout << fromLidarToGPSCur.matrix() << "\n";
    }

    // Align with GPS backbone info
    std::cout << "Align with GPS backbone info...\n";
    for (int i = 0; i < subMapRefFull->width; i++)
    {
        Eigen::Vector4d oriPoint;
        oriPoint << subMapRefFull->points[i].x, subMapRefFull->points[i].y, subMapRefFull->points[i].z, 1.0;
        Eigen::Vector4d transformedPoint = oriPoint;

        pcl::PointXYZ temp;
        subMapRefFull->points[i].x = transformedPoint[0];
        subMapRefFull->points[i].y = transformedPoint[1];
        subMapRefFull->points[i].z = transformedPoint[2];
    }
    for (int i = 0; i < subMapCurFull->width; i++)
    {
        Eigen::Vector4d oriPoint;
        oriPoint << subMapCurFull->points[i].x, subMapCurFull->points[i].y, subMapCurFull->points[i].z, 1.0;
        Eigen::Vector4d transformedPoint = fromLidarToGPSRef.inverse() * fromLidarToGPSCur * oriPoint;

        pcl::PointXYZ temp;
        subMapCurFull->points[i].x = transformedPoint[0];
        subMapCurFull->points[i].y = transformedPoint[1];
        subMapCurFull->points[i].z = transformedPoint[2];
    }

    // Try Downsample to speed up
    // Downsample
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterMap;
    // Set downsize filter
    downSizeFilterMap.setLeafSize(1.0, 1.0, 1.0);
    // Downsize pointcloud
    std::cout << "Downsize filtering submap...\n";
    downSizeFilterMap.setInputCloud(subMapRefFull);
    downSizeFilterMap.filter(*subMapRef);
    subMapRef->width = (int) subMapRef->points.size();
    subMapRef->height = 1;

    downSizeFilterMap.setInputCloud(subMapCurFull);
    downSizeFilterMap.filter(*subMapCur);
    subMapCur->width = (int) subMapCur->points.size();
    subMapCur->height = 1;

    // Align with LM
    std::cout << "Align with LM...\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr subMapCurTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < subMapCur->width; i++)
    {
        subMapCurTransformed->push_back(subMapCur->points[i]);
    }
    subMapCurTransformed->width = (int) subMapCurTransformed->points.size();
    subMapCurTransformed->height = 1;

    // Ref Points
    for (int i = 0; i < subMapRef->width; i++)
    {
        pcl::PointXYZRGB point;
        point.x = subMapRef->points[i].x;
        point.y = subMapRef->points[i].y;
        point.z = subMapRef->points[i].z;

        point.b = 255;
        point.g = 0;
        point.r = 0;

        point_cloud_ptr->push_back(point);
    }

    std::vector<bool> inliers;
    bool isConverge = false;
    int iter = 0;
    Eigen::Isometry3d fromCurToRef_estimated = Eigen::Isometry3d::Identity();
    double robustNormWidth = 3.0;

    while(!isConverge && iter < 100)
    {
        double outlierThreshold = 100;

        if(iter == 3)
            outlierThreshold = 10;

        outlierThreshold -= iter*0.5;
        outlierThreshold = outlierThreshold < 2.0 ? 2.0 : outlierThreshold;
        Eigen::Isometry3d fromCurToRef_inc = MappingICP(subMapRef, subMapCurTransformed, outlierThreshold, robustNormWidth);
        std::cout << "iter: " << iter++ << std::endl;
        std::cout << fromCurToRef_inc.matrix() << std::endl;

        if(fabs(fromCurToRef_inc.translation()[0]) < 0.001 && fabs(fromCurToRef_inc.translation()[1]) < 0.001 && fabs(fromCurToRef_inc.translation()[2]) < 0.001)
            isConverge = true;

        for (int i = 0; i < subMapCurTransformed->points.size(); i++)
        {
            Eigen::Vector4d oriPoint;
            oriPoint << subMapCurTransformed->points[i].x, subMapCurTransformed->points[i].y, subMapCurTransformed->points[i].z, 1.0;
            Eigen::Vector4d transformedPoint = fromCurToRef_inc * oriPoint;

            subMapCurTransformed->points[i].x = transformedPoint[0];
            subMapCurTransformed->points[i].y = transformedPoint[1];
            subMapCurTransformed->points[i].z = transformedPoint[2];
        }
        fromCurToRef_estimated = fromCurToRef_inc*fromCurToRef_estimated;
    }
    std::cout << "Final: "<< fromCurToRef_estimated.matrix() << std::endl;
    subMapCurTransformed->width = (int) subMapCurTransformed->points.size();
    subMapCurTransformed->height = 1;


    for (int i = 0; i < subMapCurTransformed->width; i++)
    {
        pcl::PointXYZRGB point;
        point.x = subMapCurTransformed->points[i].x;
        point.y = subMapCurTransformed->points[i].y;
        point.z = subMapCurTransformed->points[i].z;

        point.g = 255;
        point.b = 0;
        point.r = 0;

        point_cloud_ptr->push_back(point);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloud_ptr);

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}

int gpsRegistrationClosedForm()
{
    std::string finalPathString = "/home/yiluo/LIDAR/result/path.txt";
    std::string FolderPathString = "/home/yiluo/LIDAR/result/trajectory/";
    std::string resultFolderPathString = "/home/yiluo/LIDAR/result/gpsbackbone/";

    std::vector<std::string> trajectoryFileVector;
    boost::filesystem::path FolderPath(FolderPathString);
    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(FolderPath); i != boost::filesystem::directory_iterator(); i++)
    {
        if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
        {
            std::string tmp = i->path().filename().string();
            trajectoryFileVector.push_back(tmp);
        } else
            continue;
    }

    std::ofstream pathFileOut;
    pathFileOut.open(finalPathString.c_str());

    for(int i = 0; i < trajectoryFileVector.size(); i++)
    {
        std::string filename;
        filename = FolderPathString + trajectoryFileVector[i];

        const char *filename_c = filename.c_str();
        ifstream data(filename_c);
        std::string line;
        std::string timestamp;

        std::vector<double> px;
        std::vector<double> py;
        std::vector<double> pz;
        std::vector<double> gpsx;
        std::vector<double> gpsy;

        while (getline(data, line))
        {
            std::stringstream lineStream(line);
            std::string cell;
            while (getline(lineStream, cell, '\n'))
            {  //only read one row data
                int p = int(cell.find(';'));     //p store the ' ' occurs place
                if (p != std::string::npos)
                {
                    timestamp = cell.substr(10, 13);

                    //p has value
                    std::string m = cell.substr(p + 1, cell.length()); //string m = [0,p]

                    p = int(m.find(':'));
                    m = m.substr(p + 1, m.length()); //string m = [0,p]

                    p = int(m.find(','));
                    m = m.substr(p + 1, m.length()); //string m = [0,p]
                    p = int(m.find(','));
                    m = m.substr(p + 1, m.length()); //string m = [0,p]
                    p = int(m.find(','));
                    m = m.substr(p + 1, m.length()); //string m = [0,p]

                    p = int(m.find(','));
                    double x_temp = atof(m.substr(0, p).c_str());
//                    px.push_back(x_temp);
                    m = m.substr(p + 1, m.length()); //string m = [0,p]

                    p = int(m.find(','));
                    double y_temp = atof(m.substr(0, p).c_str());
//                py.push_back(y_temp);
                    m = m.substr(p + 1, m.length()); //string m = [0,p]

                    p = int(m.find(';'));
                    double z_temp = atof(m.substr(0, p).c_str());
//                pz.push_back(z_temp);
                    m = m.substr(p + 9, m.length()); //string m = [0,p]

                    p = int(m.find(','));
                    double gpsx_temp = atof(m.substr(0, p).c_str());
//                gpsx.push_back(gpsx_temp);
//
                    double gpsy_temp = atof(m.substr(p + 1, m.length()).c_str());
//                gpsy.push_back(gpsy_temp);

                    if (gpsx.size() == 0)
                    {
                        px.push_back(x_temp);
                        py.push_back(y_temp);
                        pz.push_back(z_temp);
                        gpsx.push_back(gpsx_temp);
                        gpsy.push_back(gpsy_temp);

                        std::cout << x_temp << " " << y_temp << " " << z_temp << " " << gpsx_temp << " " << gpsy_temp
                                  << std::endl;
                    } else if (fabs(gpsx_temp - gpsx[gpsx.size() - 1]) > 0.0000001 ||
                               fabs(gpsy_temp - gpsy[gpsy.size() - 1]) > 0.0000001)
                    {
                        px.push_back(x_temp);
                        py.push_back(y_temp);
                        pz.push_back(z_temp);
                        gpsx.push_back(gpsx_temp);
                        gpsy.push_back(gpsy_temp);

                        std::cout << x_temp << " " << y_temp << " " << z_temp << " " << gpsx_temp << " " << gpsy_temp
                                  << std::endl;
                    }
                }

            }
        }

        int numOfPoint = px.size();

        if(numOfPoint > 3)
        {
            cv::Mat odom_points(3, numOfPoint, CV_32F);
            cv::Mat gps_points(3, numOfPoint, CV_32F);

            for (int j = 0; j < numOfPoint; ++j)
            {
                odom_points.at<float>(0, j) = (float) px[j];
                odom_points.at<float>(1, j) = (float) py[j];
                odom_points.at<float>(2, j) = (float) pz[j];

                gps_points.at<float>(0, j) = (float) gpsx[j];
                gps_points.at<float>(1, j) = (float) gpsy[j];
                gps_points.at<float>(2, j) = 0.0;
            }

            /** Absolute Orientation Estimation**/
            cv::Mat A = odom_points.clone();
            cv::Mat B = gps_points.clone();

            int num = numOfPoint;
            // Calculate mean
            cv::Mat Ca, Cb;
            cv::reduce(A, Ca, 1, CV_REDUCE_AVG);
            cv::reduce(B, Cb, 1, CV_REDUCE_AVG);
            //    cout << "mean_odom_points: " << mean_odom_points << endl;
            //    cout << "mean_gps_points: " << mean_gps_points << endl;
            //    cv::Mat sum_odom_points;
            //    cv::reduce(odom_points, sum_odom_points, 1, CV_REDUCE_SUM);
            //    cout << "sum_odom_points: " << sum_odom_points << endl;

            // Zero-centered
            cv::Mat Ca_repeat, Cb_repeat;
            cv::repeat(Ca, 1, num, Ca_repeat);
            cv::repeat(Cb, 1, num, Cb_repeat);
            cv::Mat An = A - Ca_repeat; // odom_points_normalized
            cv::Mat Bn = B - Cb_repeat; // gps_points_normalized
            //    cout << "odom_points_normalized: " << An << endl;
            //    cout << "gps_points_normalized: " << Bn << endl;

            // Compute the quaternions
            cv::Mat M(4, 4, CV_32F, cv::Scalar(0));
            for (int i = 0; i < num; i++)
            {
                // Shortcuts
                cv::Mat a = An.col(i).clone();
                cv::Mat b = Bn.col(i).clone();
                //        cout << "a: " << a << endl;
                //        cout << "b: " << b << endl;
                float *a_data = (float *) a.data;
                float *b_data = (float *) b.data;

                // Crossproducts
                float Ma_data[16] = {0.0, -a_data[0], -a_data[1], -a_data[2],
                                     a_data[0], 0, a_data[2], -a_data[1],
                                     a_data[1], -a_data[2], 0, a_data[0],
                                     a_data[2], a_data[1], -a_data[0], 0};
                float Mb_data[16] = {0.0, -b_data[0], -b_data[1], -b_data[2],
                                     b_data[0], 0, -b_data[2], b_data[1],
                                     b_data[1], b_data[2], 0, -b_data[0],
                                     b_data[2], -b_data[1], b_data[0], 0};
                cv::Mat Ma(4, 4, CV_32F, Ma_data);
                cv::Mat Mb(4, 4, CV_32F, Mb_data);
                //        cout << "Ma: " << Ma << endl;
                //        cout << "Mb: " << Mb << endl;

                M = M + Ma.t() * Mb;
            }
            //    cout << "M: " << M << endl;
            cv::Mat eigenVal, eigenVec;
            cv::eigen(M, eigenVal, eigenVec);
            //    cout << "eigenVal: " << eigenVal << endl;
            //    cout << "eigenVec: " << eigenVec << endl;

            cv::Mat e = eigenVec.row(0).clone();
            float *e_data = (float *) e.data;
            float M1_data[16] = {e_data[0], -e_data[1], -e_data[2], -e_data[3],
                                 e_data[1], e_data[0], e_data[3], -e_data[2],
                                 e_data[2], -e_data[3], e_data[0], e_data[1],
                                 e_data[3], e_data[2], -e_data[1], e_data[0]};
            float M2_data[16] = {e_data[0], -e_data[1], -e_data[2], -e_data[3],
                                 e_data[1], e_data[0], -e_data[3], e_data[2],
                                 e_data[2], e_data[3], e_data[0], -e_data[1],
                                 e_data[3], -e_data[2], e_data[1], e_data[0]};
            cv::Mat M1(4, 4, CV_32F, M1_data);
            cv::Mat M2(4, 4, CV_32F, M2_data);
            cv::Mat R44 = M1.t() * M2;
            cv::Mat R_estimated = R44.rowRange(1, 4).colRange(1, 4);
            cout << "R estimated: " << R_estimated << endl;

            // scale s
//        float s_estimated = 0.0;
//        float a = 0.0;
//        float b = 0.0;
//        for (int i = 0; i < num; i++)
//        {
//            cv::Mat a_temp = Bn.col(i).t() * R_estimated * An.col(i);
//            //        cout << "a_temp.at<float>(0): " << a_temp.at<float>(0) << endl;
//            cv::Mat b_temp = Bn.col(i).t() * Bn.col(i);
//            a = a + a_temp.at<float>(0);
//            b = b + b_temp.at<float>(0);
//        }
//        s_estimated = b / a;

            // Do not estimate scale
            float s_estimated = 1.0;
            //    cout << "b: " << b << endl;
            //    cout << "a: " << a << endl;
            cout << "s estimated: " << s_estimated << endl;

            // translation T
            cv::Mat T_estimated = Cb - s_estimated * R_estimated * Ca;
            cout << "T estimated: " << T_estimated << endl;

            // Compute residual error
            float error = 0;
            for (int i = 0; i < num; i++)
            {
                cv::Mat d = B.col(i) - (s_estimated * R_estimated * A.col(i) + T_estimated);
                error += cv::norm(d);
            }
            cout << "error: " << error << endl;

            std::ofstream trajectoryFileOut;
            //filename = resultFolderPathString + trajectoryFileVector[i];
            filename = resultFolderPathString + timestamp + ".txt";
            trajectoryFileOut.open(filename.c_str());

            trajectoryFileOut << R_estimated.at<float>(0, 0) << " ";
            trajectoryFileOut << R_estimated.at<float>(0, 1) << " ";
            trajectoryFileOut << R_estimated.at<float>(0, 2) << " ";
            trajectoryFileOut << T_estimated.at<float>(0) << " ";

            trajectoryFileOut << R_estimated.at<float>(1, 0) << " ";
            trajectoryFileOut << R_estimated.at<float>(1, 1) << " ";
            trajectoryFileOut << R_estimated.at<float>(1, 2) << " ";
            trajectoryFileOut << T_estimated.at<float>(1) << " ";

            trajectoryFileOut << R_estimated.at<float>(2, 0) << " ";
            trajectoryFileOut << R_estimated.at<float>(2, 1) << " ";
            trajectoryFileOut << R_estimated.at<float>(2, 2) << " ";
            trajectoryFileOut << T_estimated.at<float>(2) << " ";
            trajectoryFileOut << std::endl;


            Eigen::Isometry3d fromSubmapToGPS = Eigen::Isometry3d::Identity();

            fromSubmapToGPS.matrix()(0,0) = R_estimated.at<float>(0, 0);
            fromSubmapToGPS.matrix()(0,1) = R_estimated.at<float>(0, 1);
            fromSubmapToGPS.matrix()(0,2) = R_estimated.at<float>(0, 2);
            fromSubmapToGPS.matrix()(0,3) = T_estimated.at<float>(0);

            fromSubmapToGPS.matrix()(1,0) = R_estimated.at<float>(1, 0);
            fromSubmapToGPS.matrix()(1,1) = R_estimated.at<float>(1, 1);
            fromSubmapToGPS.matrix()(1,2) = R_estimated.at<float>(1, 2);
            fromSubmapToGPS.matrix()(1,3) = T_estimated.at<float>(1);

            fromSubmapToGPS.matrix()(2,0) = R_estimated.at<float>(2, 0);
            fromSubmapToGPS.matrix()(2,1) = R_estimated.at<float>(2, 1);
            fromSubmapToGPS.matrix()(2,2) = R_estimated.at<float>(2, 2);
            fromSubmapToGPS.matrix()(2,3) = T_estimated.at<float>(2);


            // Write Points
            for (int i = 0; i < px.size(); i++)
            {
                Eigen::Vector4d srcOri, srcTransformed;
                srcOri << px[i], py[i], pz[i], 1.0;
                srcTransformed = fromSubmapToGPS*srcOri;
                pathFileOut << "Src: " << srcTransformed[0]
                            << "," << srcTransformed[1]
                            << "," << srcTransformed[2]
                            << ", Ref: " << gpsx[i] << "," << gpsy[i] << "\n";
            }
        }
    }

    pathFileOut.close();
    std::cout << "gpsRegistrationClosedForm done!\n";

    return 0;
}

void combineSubmap()
{
    std::string submapFolderPathString = "/home/yiluo/LIDAR/result/filteredsubmap/";
    std::string backboneFolderPathString = "/home/yiluo/LIDAR/result/gpsbackbone/";
    std::string wholeMapPathString = "/home/yiluo/LIDAR/result/wholeMap.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr subMapFull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr subMap(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr wholeMap(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;

    std::vector<std::string> submapFileVector;
    boost::filesystem::path submapFolderPath(submapFolderPathString);
    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(submapFolderPath); i != boost::filesystem::directory_iterator(); i++)
    {
        if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
        {
            std::string tmp = i->path().filename().string();
            //            cout << tmp << endl;
            submapFileVector.push_back(tmp);
        } else
            continue;
    }
    std::sort(submapFileVector.begin(), submapFileVector.end());

    boost::filesystem::path backboneFolderPath(backboneFolderPathString);
//    boost::unordered::unordered_map<int, std::string> backboneFileMap;
    std::map<int, std::string> backboneFileMap;
    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(backboneFolderPath); i != boost::filesystem::directory_iterator(); i++)
    {
        if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
        {
            std::string tmp = i->path().filename().string();
            int timestamp = std::atoi(tmp.substr(0, 10).c_str());
            backboneFileMap.insert(std::pair<int, std::string>(timestamp, tmp));
        } else
            continue;
    }

    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterMap;
    downSizeFilterMap.setLeafSize(5.0, 5.0, 5.0);

//    boost::unordered::unordered_map<int, std::string>::iterator it;
    std::map<int, std::string>::iterator it;
    for (int i = 0; i < submapFileVector.size(); ++i)
    {
        Eigen::Affine3d fromLidarToGPS = Eigen::Affine3d::Identity();
        int timestamp = std::atoi(submapFileVector[i].substr(0, 10).c_str());
        it = backboneFileMap.find(timestamp);
        if(it != backboneFileMap.end())
        {
            std::string filename;
            filename = backboneFolderPathString + it->second;
            std::cout << filename << std::endl;
            ifstream data(filename.c_str());
            std::string line;
            if (getline(data, line))
            {
                std::istringstream iss;
                iss.str(line);
                double tmp;

                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        iss >> tmp;
                        fromLidarToGPS(i, j) = tmp;
                    }
                }
                for (int i = 0; i < 3; ++i)
                {
                    fromLidarToGPS(3, i) = 0;
                }
                fromLidarToGPS(3, 3) = 1.0;
                std::cout << fromLidarToGPS.matrix() << "\n";;
            }
            data.close();

            std::cout << "Processing " << submapFileVector[i] << "\n";
            // Read submap
            subMapFull->clear();
            subMap->clear();
            pcl::io::loadPCDFile(submapFolderPathString + submapFileVector[i], cloud_blob);
            pcl::fromPCLPointCloud2(cloud_blob, *subMapFull); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

            // Downsize pointcloud
            downSizeFilterMap.setInputCloud(subMapFull);
            downSizeFilterMap.filter(*subMap);
            subMap->width = (int) subMap->points.size();
            subMap->height = 1;

            // Transform submap to wholemap
            for (int i = 0; i < subMap->width; i++)
            {
                Eigen::Vector4d oriPoint;
                oriPoint << subMap->points[i].x, subMap->points[i].y, subMap->points[i].z, 1.0;
                Eigen::Vector4d transformedPoint = fromLidarToGPS * oriPoint;
                subMap->points[i].x = transformedPoint[0];
                subMap->points[i].y = transformedPoint[1];
                subMap->points[i].z = transformedPoint[2];
            }
            // Concatenate into wholemap
            *wholeMap += *subMap;
        }
    }

    // Store wholeMap
    wholeMap->width = (int) wholeMap->points.size();
    wholeMap->height = 1;
    std::cout << "Writing wholeMap..." << std::endl;
    pcl::io::savePCDFileASCII(wholeMapPathString.c_str(), *wholeMap);
}

void downSample(std::string fileName, std::string oriPath, std::string targetPath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInput(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInputSub(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudFilteredSub(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;

    std::cout << "Loading " << fileName << "..." << std::endl;
    pcl::io::loadPCDFile(oriPath + fileName, cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *laserCloudInput); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

    int totalNumOfPoints = laserCloudInput->points.size();
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterMap;
    int start = 0;
    int increment = totalNumOfPoints/100;
    int end = increment;

    std::cout << "Downsize filtering...\n";
    while(end <= totalNumOfPoints)
    {
        laserCloudInputSub->clear();
        laserCloudFilteredSub->clear();
        for(int j = start; j < end; j++)
        {
            laserCloudInputSub->push_back(laserCloudInput->points[j]);
        }
        start = end;
        end += increment;

        // Set downsize filter
        downSizeFilterMap.setLeafSize(3.0, 3.0, 3.0);
        // Downsize pointcloud

        downSizeFilterMap.setInputCloud(laserCloudInputSub);
        downSizeFilterMap.filter(*laserCloudFilteredSub);

        *laserCloudFiltered += *laserCloudFilteredSub;
        laserCloudFiltered->width = (int) laserCloudFiltered->points.size();
        laserCloudFiltered->height = 1;
    }
    std::cout << "Writing into disk..." << std::endl;
    pcl::io::savePCDFileASCII(targetPath + fileName, *laserCloudFiltered);
}

void downSampleSubmaps()
{
    std::string FolderPathString = "/home/yiluo/LIDAR/result/submap/";
    std::string resultFolderPathString = "/home/yiluo/LIDAR/result/filteredsubmap/";

    std::vector<std::string> trajectoryFileVector;
    boost::filesystem::path FolderPath(FolderPathString);
    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(FolderPath); i != boost::filesystem::directory_iterator(); i++)
    {
        if (!boost::filesystem::is_directory(i->path())) //we eliminate directories
        {
            std::string tmp = i->path().filename().string();
            //            cout << tmp << endl;
            trajectoryFileVector.push_back(tmp);
        } else
            continue;
    }

    for(int i = 0; i < trajectoryFileVector.size(); i++)
    {
        downSample(trajectoryFileVector[i], FolderPathString, resultFolderPathString);
    }

    std::cout << "downSampleSubmaps done!\n";
}

void printHelp()
{
    std::cout << "Command: SubmapUtils arg1\n";
    std::cout << "Example:\n";
    std::cout << "\tSubmapUtils 1: downSampleSubmaps\n";
    std::cout << "\tSubmapUtils 2: gpsRegistrationClosedForm\n";
    std::cout << "\tSubmapUtils 3: combineSubmap\n";
    std::cout << "\tSubmapUtils 4: alignTwoTracks\n";
}

int main(int argc,char *argv[])
{
    if (argc < 2)
    {
        printHelp();
        return 1;

    } else
    {
        switch (atoi(argv[1]))
        {
            case 1:
                downSampleSubmaps();
                break;

            case 2:
                gpsRegistrationClosedForm();
                break;

            case 3:
                combineSubmap();
                break;

            case 4:
                alignTwoTracks();
                break;

            default:
                printHelp();
                return 1;
        }
    }

    return 0;
}


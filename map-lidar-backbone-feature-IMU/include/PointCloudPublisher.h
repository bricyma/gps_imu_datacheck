/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PointCloudPublisher.h
 * Author: yiluo
 *
 * Created on December 15, 2016, 3:03 PM
 */

#ifndef POINTCLOUDPUBLISHER_H
#define POINTCLOUDPUBLISHER_H

/*
  Include directly the different
  headers from cppconn/ and mysql_driver.h + mysql_util.h
  (and mysql_connection.h). This will reduce your build time!
 */
#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <cmath>
#include <vector>

#include <common.h>
#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <iomanip>

using namespace std;

// For synchronization
static bool getMappingDoneSignal = false;
static int frameCount = 0;

// For Reset System
static ros::Publisher pubResetMappingSignal;

// Define Class CSVRow for reading Point from CVS
class CSVRow
{
public:

    std::string const& operator[](std::size_t index) const
    {
        return m_data[index];
    }

    std::size_t size() const
    {
        return m_data.size();
    }

    void readNextRow(std::istream& str)
    {
        std::string line;
        std::getline(str, line);

        std::stringstream lineStream(line);
        std::string cell;

        m_data.clear();
        while (std::getline(lineStream, cell, ','))
        {
            m_data.push_back(cell);
        }
    }
private:
    std::vector<std::string> m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}
// End definition of Class CSVRow 

// Functions

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes)
{
    getMappingDoneSignal = true;
}

void resetSignalHandler(const sensor_msgs::PointCloud2ConstPtr& submapPointCloudMsg)
{
    frameCount = 0;
}

void printHelp()
{
    std::cout << "Command: PointCloudPublisher arg1 arg2\n";
    std::cout << "Example:\n";
    std::cout << "PointCloudPublisher 1479967790 1479967870: publish lidar, gps... frame from time 1479967790 to 1479967870\n";
}


#endif /* POINTCLOUDPUBLISHER_H */


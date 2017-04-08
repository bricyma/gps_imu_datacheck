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

#include "../include/PointCloudPublisher.h"

int main(int argc, char** argv)
{


    if (argc < 3)
    {
        printHelp();
        return 1;
    }
    int startTime = atoi(argv[1]);
    int endTime = atoi(argv[2]);
    if (endTime < startTime)
    {
        printHelp();
        return 1;
    }

    /* Set Up Global Variable Start*/
    // For synchronization
    getMappingDoneSignal = false;
    frameCount = 0;
    /* Set Up Global Variable End*/

    ros::init(argc, argv, "PointCloudPublisher");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
            ("/velodyne_cloud_registered", 1, laserCloudFullResHandler);

    ros::Subscriber subResetSignal = nh.subscribe<sensor_msgs::PointCloud2>
            ("/reset_signal", 2, resetSignalHandler);

    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    ros::Publisher pubGPSMsg = nh.advertise<sensor_msgs::NavSatFix>("/gps_raw", 2);
    ros::Publisher pubSpeedMsg = nh.advertise<sensor_msgs::NavSatFix>("/speed_raw", 2);
    pubResetMappingSignal = nh.advertise<sensor_msgs::PointCloud2>("/reset_mapping_signal", 2);

    cout << "Preparing data...\n";

    // Initialize connector
    sql::Driver *driver;
    sql::Connection *con;
    sql::Statement *stmt;

    sql::ResultSet *res;
    sql::PreparedStatement *pstmt;
    driver = get_driver_instance();
    con = driver->connect("tcp://192.168.1.135:3306", "tusimple", "tusimple");
    stmt = con->createStatement();
    stmt->execute("USE dope");

    // Calculate Fix GPS Pos 
    double startLat = 39.9080145000;
    double startLon = 116.5321005000;
    double er = 6378137.0; //6378137.0
    double pi = 3.14159265359;
    double scale = cos(startLat * pi / 180.);
    double fixedGPS_X = scale * startLon * pi * er / 180.0;
    double fixedGPS_Y = scale * er * log(tan((90.0 + startLat) * pi / 360.0));

    //    pstmt = con->prepareStatement("SELECT server_time, latitude, longitude, altitude, points FROM mergetest where server_time between 1479882100 and 1479882120"); // 1: 1479879903, 2: 1479882557
    //    pstmt = con->prepareStatement("SELECT server_time, latitude, longitude, altitude, points FROM mergetest_path where server_time between 1479885000 and 1479885080"); // 1479879908 1479879903 1479891000

    pcl::PointCloud<pcl::PointXYZ> laserCloudOut;

    //    int startTime = 1479967790; //1479881230 1479883500 1479887928 1479887910
    //    int endTime = 1479967870; // 1479896000
    int nowTime = startTime;
    double time;
    int increment = 10;
    int timeCounter = 0;

    while (nowTime < endTime)
    {
        char queryCommand[256];
        sprintf(queryCommand, "SELECT server_time, latitude, longitude, altitude, points FROM mergetest_path where server_time between %d and %d", nowTime, nowTime + increment);

        // Get speed first
        pstmt = con->prepareStatement(queryCommand);
        res = pstmt->executeQuery();

        std::vector<double> latVec;
        std::vector<double> lonVec;
        timeCounter = 0;
        while (res->next())
        {
            timeCounter++;
            double latitude = res->getDouble("latitude");
            double longitude = res->getDouble("longitude");

            latVec.push_back(latitude);
            lonVec.push_back(longitude);
        }

        int vecSize = latVec.size();
        double startGPS_X = scale * lonVec[0] * pi * er / 180.0;
        double startGPS_Y = scale * er * log(tan((90.0 + latVec[0]) * pi / 360.0));
        double endGPS_X = scale * lonVec[vecSize - 1] * pi * er / 180.0;
        double endGPS_Y = scale * er * log(tan((90.0 + latVec[vecSize - 1]) * pi / 360.0));
        double difX = endGPS_X - startGPS_X;
        double difY = endGPS_Y - startGPS_Y;

        double meanSpeed = sqrt(difX * difX + difY * difY) / (timeCounter);
        meanSpeed *= 1.0;

        cout << "Speed Prior: " << meanSpeed << endl;
        cout << "Estimated trajectory length: " << meanSpeed * timeCounter << endl;
        cout << setprecision(10) << "Start: (" << startGPS_X << "," << startGPS_Y << ")\n";
        cout << setprecision(10) << "End: (" << endGPS_X << "," << endGPS_Y << ")\n";
        cout << setprecision(10) << "Dif: (" << endGPS_X - startGPS_X << "," << endGPS_Y - startGPS_Y << ")\n";

        // Publish PointCloud and GPS
        sprintf(queryCommand, "SELECT server_time, latitude, longitude, altitude, points FROM mergetest_path where server_time between %d and %d", nowTime, nowTime + increment);
        nowTime += increment;
        // Get speed first
        pstmt = con->prepareStatement(queryCommand);
        res = pstmt->executeQuery();

        while (res->next())
        {
            time = res->getDouble("server_time"); //server_time latitude
            //        printf("server_time=%.10f ", time);

            double latitude = res->getDouble("latitude");
            //        printf("latitude=%.10f ", latitude);
            //        cout << "latitude: " << latitude << " ";

            double longitude = res->getDouble("longitude");
            //        cout << "longitude: " << longitude << " ";
            //        printf("longitude=%.10f ", longitude);

            double altitude = res->getDouble("altitude");
            //        cout << "longitude: " << longitude << " ";
            //        printf("longitude=%.10f ", longitude);

            string points_str = res->getString("points");
            //        cout << "strLen: " << points_str.size() << " ";
            cout << "Path: (" << points_str << ")\n";

            CSVRow row;
            std::ifstream file(points_str.c_str());
            laserCloudOut.clear();
            while (file >> row)
            {
                pcl::PointXYZ point;
                point.x = boost::lexical_cast<double>(row[0]);
                point.y = boost::lexical_cast<double>(row[1]);
                point.z = boost::lexical_cast<double>(row[2]);
                laserCloudOut.points.push_back(point);
            }
            file.close();
            laserCloudOut.width = (int) laserCloudOut.points.size();
            laserCloudOut.height = 1;

            // Pub Pointcloud
            cout << "Publishing " << setprecision(10) << time << " , Timestamp: " << time - floor(time) << ", Total " << laserCloudOut.width << " Points in PointCloud.\n";
            sensor_msgs::PointCloud2 laserCloudOut2;
            pcl::toROSMsg(laserCloudOut, laserCloudOut2);
            laserCloudOut2.header.stamp = ros::Time(time);
            laserCloudOut2.header.frame_id = "velodyne";
            pubLaserCloud.publish(laserCloudOut2);

            // Pub GPS
            sensor_msgs::NavSatFix gpsMsg;
            gpsMsg.header.stamp = ros::Time(time);
            gpsMsg.header.frame_id = "GPS";
            gpsMsg.latitude = latitude;
            gpsMsg.longitude = longitude;
            gpsMsg.altitude = altitude;
            pubGPSMsg.publish(gpsMsg);

            // Pub GPS
            sensor_msgs::NavSatFix speedMsg;
            speedMsg.header.stamp = ros::Time(time);
            speedMsg.header.frame_id = "Speed";
            speedMsg.latitude = meanSpeed;
            speedMsg.longitude = meanSpeed;
            speedMsg.altitude = meanSpeed;
            pubSpeedMsg.publish(speedMsg);

            clock_t t;
            t = clock();
            //        usleep(1000 * 1000); // sleep for a short time
            if (frameCount >= 3)
                while (!getMappingDoneSignal)
                {
                    ros::spinOnce();
                    if ((clock() - t) > 30.0 * CLOCKS_PER_SEC) getMappingDoneSignal = true;
                }
            frameCount++;
            getMappingDoneSignal = false;


            if (!ros::ok()) return 0;
        }

        cout << "Estimated trajectory length: " << meanSpeed * timeCounter << endl;
        cout << "Start: (" << startGPS_X << "," << startGPS_Y << ")\n";
        cout << "End: (" << endGPS_X << "," << endGPS_Y << ")\n";
        cout << "Dif: (" << endGPS_X - startGPS_X << "," << endGPS_Y - startGPS_Y << ")\n";


        // Send signal
        //        pcl::PointCloud<PointType> resetMappingSignalPointCloud;
        //        resetMappingSignalPointCloud.clear();
        //        PointType resetPoint;
        //        resetPoint.x = -1.0;
        //        resetPoint.y = -1.0;
        //        resetPoint.z = -1.0;
        //        resetMappingSignalPointCloud.push_back(resetPoint);
        //        sensor_msgs::PointCloud2 resetSignalPointCloudMsg;
        //        pcl::toROSMsg(resetMappingSignalPointCloud, resetSignalPointCloudMsg);
        //        resetSignalPointCloudMsg.header.stamp = ros::Time().fromSec(time);
        //        resetSignalPointCloudMsg.header.frame_id = "/camera_init";
        //        pubResetMappingSignal.publish(resetSignalPointCloudMsg);
    }

    delete res;
    delete stmt;
    delete con;

    return 0;
}
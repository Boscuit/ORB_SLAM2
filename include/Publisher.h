// #ifndef PUBLISHER_H
// #define PUBLISHER_H
//
// #include "System.h"
// #include "MapDrawer.h"
// #include <ros/ros.h>
// 
// using namespace std;
//
//
// class MapDrawer;
// class System;
//
// class Publisher
// {
// public:
//     Publisher(System* pSystem, MapDrawer* pMapDrawer);
//
//     void Run();
//
// private:
//
//   ORB_SLAM2::System* mpSystem;
//
//   ORB_SLAM2::MapDrawer* mpMapDrawer;
//
//   int count = 0;
//   const int mfootprint; // constant variable can only be initialized in the constructor
//   cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
//   cv::Mat Tcv = (cv::Mat_<float>(4,4) << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0);
//   vector<float> vPubPose{0,0,0,0,0,0,1};
//   ros::NodeHandle n_;
//   ros::Publisher pub_;
//   ros::Publisher mark_;
//   ros::Subscriber sub_;
//
// };
//
// #endif // PUBLISHER_H

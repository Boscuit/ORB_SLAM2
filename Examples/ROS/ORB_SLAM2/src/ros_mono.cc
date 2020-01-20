/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM) //constructor
    {
      pub_ = n_.advertise<geometry_msgs::PoseStamped>("current_pose",1);
      sub_ = n_.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,this);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg); //callback

    ORB_SLAM2::System* mpSLAM;

private:
    cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
    vector<float> vPubPose{0,0,0,0,0,0,1};
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};

// class PosePublisher
// {
// public:
//     PosePublisher(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
//
//     void PubPose(const sensor_msgs::ImageConstPtr& msg);
//
//     ORB_SLAM2::System* mpSLAM;
// };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);
    // PosePublisher pps(&SLAM);


    // ros::NodeHandle nodeHandler;
    // //Publisher
    // ros::Publisher current_pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("current_pose",1);
    //
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    // // ros::Subscriber pose_sub = nodeHandler.subscribe("/camera/image_raw", 1, &PosePublisher::PubPose,&pps);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    if (!Tcw.empty())
    {
      Twc = mpSLAM->Tcw2Twc(Tcw);
      vPubPose = mpSLAM->Twc2vPubPose(Twc);
    }
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = ros::Time::now();;
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = vPubPose[0];
    current_pose.pose.position.y = vPubPose[1];
    current_pose.pose.position.z = vPubPose[2];
    current_pose.pose.orientation.x = vPubPose[3];
    current_pose.pose.orientation.y = vPubPose[4];
    current_pose.pose.orientation.z = vPubPose[5];
    current_pose.pose.orientation.w = vPubPose[6];
    pub_.publish(current_pose);
    // cout << "vPubPose:[";
    // for (int i=0;i<vPubPose.size();i++)
    // {
    //   cout << setprecision(4) << vPubPose[i];
    //   cout << ", ";
    // }
    // cout << ']' << endl;
    // cout << "TrackMonocular_Pose:" << endl << Twc << endl;

}

// void PosePublisher::PubPose(const sensor_msgs::ImageConstPtr& msg)
// {
//   // cout << "PubPose" << endl;
//   // mpSLAM->GetCurrentCameraPose();
//   // geometry_msgs::PoseStamped current_pose;
//   // current_pose.header.stamp = ros::Time::now();;
//   // current_pose.header.frame_id = "map";
//   // current_pose.pose.position.x = -0.02901135;
//   // current_pose.pose.position.y = 0.03019926;
//   // current_pose.pose.position.z = -0.05879022;
//   // current_pose.pose.orientation.x = 0.7153;
//   // current_pose.pose.orientation.y = -0.0051;
//   // current_pose.pose.orientation.z = 0.0278;
//   // current_pose.pose.orientation.w = 0.6983;
//   // current_pose_pub.publish(current_pose);
// }

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
#include "visualization_msgs/Marker.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM,int footprint):mpSLAM(pSLAM),mfootprint(footprint) //constructor ：mpSLAM is initialized as pSLAM.
    {
      pub_ = n_.advertise<geometry_msgs::PoseStamped>("current_pose",1);
      mark_ = n_.advertise<visualization_msgs::Marker>("pose_marker", 1);
      key_ = n_.advertise<visualization_msgs::Marker>("keypose_marker", 1);
      sub_ = n_.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,this);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg); //callback

    ORB_SLAM2::System* mpSLAM;

private:
    int count = -1;
    const int mfootprint; // constant variable can only be initialized in the constructor
    cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
    cv::Mat Tcv = (cv::Mat_<float>(4,4) << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0);
    vector<float> vPubPose{0,0,0,0,0,0,1};
    vector<float> vPubKeyPose{0,0,0,0,0,0,1};
    vector<cv::Mat> vKeyPose;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher mark_;
    ros::Publisher key_;
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

    ImageGrabber igb(&SLAM,100);
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
      // cv::Mat Twv = Tcv*Twc;
      vPubPose = mpSLAM->Twc2vPubPose(Twc);
      vKeyPose = mpSLAM->GetKeyCameraPoseVector();
    }

    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = ros::Time::now();;
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = vPubPose[0];
    current_pose.pose.position.y = vPubPose[2];
    current_pose.pose.position.z = -vPubPose[1];
    current_pose.pose.orientation.x = vPubPose[3];
    current_pose.pose.orientation.y = vPubPose[5];
    current_pose.pose.orientation.z = -vPubPose[4];
    current_pose.pose.orientation.w = vPubPose[6];
    pub_.publish(current_pose);


    // if (count<mfootprint) count++;
    // else count = 1;
    count ++;

    /*---------------current pose marker---------------*/
    visualization_msgs::Marker pose_marker;
    pose_marker.header.stamp = ros::Time::now();
    pose_marker.header.frame_id = "map";
    pose_marker.ns = "basic_shapes";
    pose_marker.id = count;
    pose_marker.type = visualization_msgs::Marker::CUBE;
    pose_marker.action = visualization_msgs::Marker::ADD;

    pose_marker.pose.position.x = vPubPose[0];
    pose_marker.pose.position.y = vPubPose[2];
    pose_marker.pose.position.z = -vPubPose[1];
    pose_marker.pose.orientation.x = vPubPose[3];
    pose_marker.pose.orientation.y = vPubPose[5];
    pose_marker.pose.orientation.z = -vPubPose[4];
    pose_marker.pose.orientation.w = vPubPose[6];
    pose_marker.scale.x = 0.005;
    pose_marker.scale.y = 0.02;
    pose_marker.scale.z = 0.005;
    pose_marker.color.r = 1.0f;
    pose_marker.color.g = 0.0f;
    pose_marker.color.b = 0.0f;
    pose_marker.color.a = 0.5;

    if (!mpSLAM->isClear() || count < 1) //Reset or initialze
    {
      pose_marker.action = visualization_msgs::Marker::DELETEALL;
      key_.publish(pose_marker);//Publish a dummy message to clear the keypose marker
      vKeyPose.clear();
    }
    mark_.publish(pose_marker);

    /*---------------key frame pose marker---------------*/
    visualization_msgs::Marker keypose_marker;
    for (int i = 0; i < vKeyPose.size(); i++)
    {
      vPubKeyPose = mpSLAM->Twc2vPubPose(vKeyPose[i]);
      keypose_marker.header.stamp = ros::Time::now();
      keypose_marker.header.frame_id = "map";
      keypose_marker.ns = "basic_shapes";
      keypose_marker.id = i;
      keypose_marker.type = visualization_msgs::Marker::CUBE;
      keypose_marker.action = visualization_msgs::Marker::ADD;

      keypose_marker.pose.position.x = vPubKeyPose[0];
      keypose_marker.pose.position.y = vPubKeyPose[2];
      keypose_marker.pose.position.z = -vPubKeyPose[1];
      keypose_marker.pose.orientation.x = vPubKeyPose[3];
      keypose_marker.pose.orientation.y = vPubKeyPose[5];
      keypose_marker.pose.orientation.z = -vPubKeyPose[4];
      keypose_marker.pose.orientation.w = vPubKeyPose[6];
      keypose_marker.scale.x = 0.02;
      keypose_marker.scale.y = 0.005;
      keypose_marker.scale.z = 0.02;
      keypose_marker.color.r = 0.0f;
      keypose_marker.color.g = 0.0f;
      keypose_marker.color.b = 1.0f;
      keypose_marker.color.a = 0.5;

      key_.publish(keypose_marker);
    }



}

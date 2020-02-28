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
#include<string>
#include<chrono>

#include<ros/ros.h>
#include<tf2_ros/transform_broadcaster.h>
#include<cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM,int footprint):mpSLAM(pSLAM),mfootprint(footprint) //constructor ：mpSLAM is initialized as pSLAM.
    {
      mark_ = n_.advertise<visualization_msgs::Marker>("pose_marker", 1);
      path_ = n_.advertise<nav_msgs::Path>("Trajectory",1);
      key_ = n_.advertise<visualization_msgs::Marker>("keypose_marker", 1);
      sub_ = n_.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,this);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg); //callback

    ORB_SLAM2::System* mpSLAM;

private:
    int count = -1;
    const int mfootprint; // constant variable can only be initialized in the constructor
    cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
    cv::Mat Twv = (cv::Mat_<float>(4,4) << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1);
    vector<float> vPubPose{0,0,0,0,0,0,1};
    vector<float> vPubKeyPose{0,0,0,0,0,0,1};
    vector<cv::Mat> vKeyPose;
    nav_msgs::Path trajectory;//contains a vector of PoseStamped always needed to be kept.
    tf2_ros::TransformBroadcaster tf2_;
    ros::NodeHandle n_;
    ros::Publisher mark_;
    ros::Publisher path_;
    ros::Publisher key_;
    ros::Subscriber sub_;

};


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
    // cout<<"main strVocFile address: " << &argv[1] <<endl;
    // cout<<"main strSettingsFile address: " << &argv[2]<<endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM,100);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM2("KeyFrameTrajectory.txt","KeyFrameKeyPoints.txt",
    "KeyFrameDescriptor.txt","KeyFrameFeatureVector.txt","KeyFrameBowVector.txt",
    "KeyFramevInvertedFile.txt","MapPointsLocationFile.txt","MapPointsDescritorFile.txt");
    cout<<"Save"<<endl;

    ros::shutdown();
    cout<<"ros down"<<endl;

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
      Twc = mpSLAM->InverseT(Tcw);
      cv::Mat Tvc = Twv.t()*Twc*Twv;//T of c2 based on viewer frame
      vPubPose = mpSLAM->Twc2vPubPose(Tvc);
      vKeyPose = mpSLAM->GetKeyCameraPoseVector();
    }

    /*--------------Current Pose with tf2---------------*/
    geometry_msgs::TransformStamped current_tf;
    current_tf.header.stamp = ros::Time::now();
    current_tf.header.frame_id = "map";
    current_tf.child_frame_id = "Camera";
    current_tf.transform.translation.x = vPubPose[0];
    current_tf.transform.translation.y = vPubPose[1];
    current_tf.transform.translation.z = vPubPose[2];
    current_tf.transform.rotation.x = vPubPose[3];
    current_tf.transform.rotation.y = vPubPose[4];
    current_tf.transform.rotation.z = vPubPose[5];
    current_tf.transform.rotation.w = vPubPose[6];
    tf2_.sendTransform(current_tf);


    /*---------Current Pose with PoseStamped-------*/
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

    /*---------------Trajectory-------------------*/
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = "map";
    trajectory.poses.push_back(current_pose);

    if (!mpSLAM->isClear() || vKeyPose.size()<1) //Reset or initialze
    {
      vKeyPose.clear();
      trajectory.poses.clear();
    }
    path_.publish(trajectory);


    /*---------key frame index marker------------*/
    visualization_msgs::Marker keypose_index;
    keypose_index.header.stamp = ros::Time::now();
    keypose_index.header.frame_id = "map";
    keypose_index.ns = "keypose_index";
    keypose_index.id = 0;//refresh every time
    keypose_index.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    keypose_index.action = visualization_msgs::Marker::ADD;
    keypose_index.scale.z = 0.03;
    keypose_index.color.r = 0.0f;
    keypose_index.color.g = 0.0f;
    keypose_index.color.b = 1.0f;
    keypose_index.color.a = 1.0;

    /*---------------key frame pose marker---------------*/
    visualization_msgs::Marker keypose_marker;
    keypose_marker.header.stamp = ros::Time::now();
    keypose_marker.header.frame_id = "map";
    keypose_marker.ns = "keypose_marker";
    keypose_marker.id = 0;//refresh every time
    keypose_marker.type = visualization_msgs::Marker::LINE_LIST;
    keypose_marker.action = visualization_msgs::Marker::ADD;
    keypose_marker.scale.x = 0.003;
    keypose_marker.color.r = 0.0f;
    keypose_marker.color.g = 0.0f;
    keypose_marker.color.b = 1.0f;
    keypose_marker.color.a = 0.5;
    for (size_t i = 0; i < vKeyPose.size(); i++)
    {
      cv::Mat keyTwc = vKeyPose[i];
      cv::Mat keyTvc = Twv.t()*keyTwc*Twv;//T of c2 based on viewer frame
      vPubKeyPose = mpSLAM->Twc2vPubPose(keyTvc);

      float s = 0.1;//scales of the marker
      cv::Mat fr = keyTvc*(cv::Mat_<float>(4,5) << 0, 0, 0, 0, -0.5,
                                                  1, -1, -1, 1, 0,
                                                  1, 1, -1, -1, 0,
                                                  1/s, 1/s, 1/s, 1/s ,1/s)*s;
      geometry_msgs::Point p;
      vector<int> index = {0,1,1,2,2,3,3,0,4,0,4,1,4,2,4,3};
      for (int j = 0; j < 16; j++)
      {
        p.x = fr.at<float>(0,index[j]);
        p.y = fr.at<float>(1,index[j]);
        p.z = fr.at<float>(2,index[j]);
        keypose_marker.points.push_back(p);
      }
      cv::Mat text = keyTvc*(cv::Mat_<float>(4,1) << 0, 0, s, 1);
      keypose_index.id = i;//index in vKeyPose
      keypose_index.pose.position.x = text.at<float>(0);
      keypose_index.pose.position.y = text.at<float>(1);
      keypose_index.pose.position.z = text.at<float>(2);
      keypose_index.pose.orientation.x = 0.0;
      keypose_index.pose.orientation.y = 0.0;
      keypose_index.pose.orientation.z = 0.0;
      keypose_index.pose.orientation.w = 1.0;
      keypose_index.text = to_string(i);
      key_.publish(keypose_index);

    }

    key_.publish(keypose_marker);

    // /*---------------key frame pose marker---------------*/
    // visualization_msgs::Marker keypose_marker;
    // keypose_marker.action = visualization_msgs::Marker::DELETEALL;
    // key_.publish(keypose_marker);//Publish a dummy message to clear the keypose marker
    // for (unsigned i = 0; i < vKeyPose.size(); i++)
    // {
    //   vPubKeyPose = mpSLAM->Twc2vPubPose(vKeyPose[i]);
    //   keypose_marker.header.stamp = ros::Time::now();
    //   keypose_marker.header.frame_id = "map";
    //   keypose_marker.ns = "basic_shapes";
    //   keypose_marker.id = i;
    //   keypose_marker.type = visualization_msgs::Marker::CUBE;
    //   keypose_marker.action = visualization_msgs::Marker::ADD;
    //
    //   keypose_marker.pose.position.x = vPubKeyPose[0];
    //   keypose_marker.pose.position.y = vPubKeyPose[2];
    //   keypose_marker.pose.position.z = -vPubKeyPose[1];
    //   keypose_marker.pose.orientation.x = vPubKeyPose[3];
    //   keypose_marker.pose.orientation.y = vPubKeyPose[5];
    //   keypose_marker.pose.orientation.z = -vPubKeyPose[4];
    //   keypose_marker.pose.orientation.w = vPubKeyPose[6];
    //   keypose_marker.scale.x = 0.1;
    //   keypose_marker.scale.y = 0.01;
    //   keypose_marker.scale.z = 0.1;
    //   keypose_marker.color.r = 0.0f;
    //   keypose_marker.color.g = 0.0f;
    //   keypose_marker.color.b = 1.0f;
    //   keypose_marker.color.a = 0.5;
    //
    //   key_.publish(keypose_marker);
    // }



}

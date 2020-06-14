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
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM,int footprint):mpSLAM(pSLAM),mfootprint(footprint) //constructor ：mpSLAM is initialized as pSLAM.
    {
      path_ = n_.advertise<nav_msgs::Path>("Trajectory",1);
      key_ = n_.advertise<visualization_msgs::Marker>("keypose_marker", 1);
      gtpath_ = n_.advertise<nav_msgs::Path>("GroundTruthPath",1);
      lgtpath_ = n_.advertise<nav_msgs::Path>("LastGroundTruthPath",1);
      sim_ = n_.advertise<visualization_msgs::Marker>("similarity_marker", 1);
      sub_ = n_.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,this);
      subGT_ = n_.subscribe("/vicon/firefly_sbx/firefly_sbx", 1, &ImageGrabber::ShowGroundTruth,this);
      subRR_ = n_.subscribe("/RequestRecord", 1, &ImageGrabber::SetRecord,this);
      subR_ = n_.subscribe("/Request", 1, &ImageGrabber::LoadImgs,this);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg); //callback

    void ShowGroundTruth(const geometry_msgs::TransformStamped& tfs);

    void SetRecord(const std_msgs::Bool& brr);

    void LoadImgs(const std_msgs::Int8& nr);

    void UpdateNodeHandle(ros::NodeHandle &n);

    ORB_SLAM2::System* mpSLAM;

    ros::NodeHandle n_;//change to public at May 28

private:
    int nImgsCount = 0;
    float offset = 0;//offset of last groundtruth for visualization
    const int mfootprint; // constant variable can only be initialized in the constructor
    cv::Mat Twv = (cv::Mat_<float>(4,4) << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1);
    vector<float> mvPubMapOrigin{0,0,0,0,0,0,1};
    vector<float> mvPubCurrentGT{0,0,0,0,0,0,1};
    vector<float> mvPubPose{0,0,0,0,0,0,1};
    vector<float> mvPubKeyPose{0,0,0,0,0,0,1};
    vector<cv::Mat> vKeyPose;
    nav_msgs::Path trajectory;//contains a vector of PoseStamped always needed to be kept.
    nav_msgs::Path GroundTruthPath;
    nav_msgs::Path LastGroundTruthPath;
    tf2_ros::TransformBroadcaster tf2_;
    ros::Publisher path_;
    ros::Publisher key_;
    ros::Publisher gtpath_;
    ros::Publisher lgtpath_;
    ros::Publisher sim_;
    ros::Subscriber sub_;
    ros::Subscriber subGT_;
    ros::Subscriber subRR_;
    ros::Subscriber subR_;

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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM,100);

    igb.UpdateNodeHandle(igb.n_);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryEuRoc("GroundTruth.csv","KeyFrameTrajectory.txt","KeyFrameKeyPointsUn.txt","KeyFrameKeyPoints.txt",
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
      cv::Mat Twc = mpSLAM->InverseT(Tcw);
      cv::Mat Tvc = Twv.t()*Twc*Twv;//T of c2 based on viewer frame(Twv=Tc1c2)
      mvPubPose = mpSLAM->Twc2sevenD(Tvc);
      vKeyPose = mpSLAM->GetKeyCameraPoseVector();//vector<cv::Mat> base on world
    }

    //align map to groundtruth in world frame
    geometry_msgs::TransformStamped map_origin;
    map_origin.header.stamp = ros::Time::now();
    map_origin.header.frame_id = "world";
    map_origin.child_frame_id = "map";
    map_origin.transform.translation.x = mvPubMapOrigin[0];
    map_origin.transform.translation.y = mvPubMapOrigin[1];
    map_origin.transform.translation.z = mvPubMapOrigin[2];
    map_origin.transform.rotation.x = mvPubMapOrigin[3];
    map_origin.transform.rotation.y = mvPubMapOrigin[4];
    map_origin.transform.rotation.z = mvPubMapOrigin[5];
    map_origin.transform.rotation.w = mvPubMapOrigin[6];
    tf2_.sendTransform(map_origin);

    /*--------------Current Pose with tf2---------------*/
    geometry_msgs::TransformStamped current_tf;
    current_tf.header.stamp = ros::Time::now();
    current_tf.header.frame_id = "map";
    current_tf.child_frame_id = "Camera";
    current_tf.transform.translation.x = mvPubPose[0];
    current_tf.transform.translation.y = mvPubPose[1];
    current_tf.transform.translation.z = mvPubPose[2];
    current_tf.transform.rotation.x = mvPubPose[3];
    current_tf.transform.rotation.y = mvPubPose[4];
    current_tf.transform.rotation.z = mvPubPose[5];
    current_tf.transform.rotation.w = mvPubPose[6];
    tf2_.sendTransform(current_tf);


    /*---------Current Pose with PoseStamped-------*/
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = mvPubPose[0];
    current_pose.pose.position.y = mvPubPose[1];
    current_pose.pose.position.z = mvPubPose[2];
    current_pose.pose.orientation.x = mvPubPose[3];
    current_pose.pose.orientation.y = mvPubPose[4];
    current_pose.pose.orientation.z = mvPubPose[5];
    current_pose.pose.orientation.w = mvPubPose[6];

    /*---------------Trajectory-------------------*/
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = "map";
    trajectory.poses.push_back(current_pose);

    if (!mpSLAM->isClear() || vKeyPose.size()<1) //Reset or initialze
    {
      vKeyPose.clear();
      trajectory.poses.clear();
      current_pose.pose.position.x = 0;
      current_pose.pose.position.y = 0;
      current_pose.pose.position.z = 0;
      current_pose.pose.orientation.x = 0;
      current_pose.pose.orientation.y = 0;
      current_pose.pose.orientation.z = 0;
      current_pose.pose.orientation.w = 1;
      trajectory.poses.push_back(current_pose);
      mvPubMapOrigin = mvPubCurrentGT;//update map origin to current groundtruth
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
      mvPubKeyPose = mpSLAM->Twc2sevenD(keyTvc);

      float s = 0.1;//scales of the marker
      cv::Mat fr = keyTvc*(cv::Mat_<float>(4,5) << 0.5, 0.5, 0.5, 0.5, 0,
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


}


void ImageGrabber::ShowGroundTruth(const geometry_msgs::TransformStamped& tfs)
{
  geometry_msgs::PoseStamped GroundTruth_pose;
  GroundTruth_pose.header.stamp = tfs.header.stamp;
  GroundTruth_pose.header.frame_id = "world";
  GroundTruth_pose.pose.position.x = tfs.transform.translation.x;
  GroundTruth_pose.pose.position.y = tfs.transform.translation.y;
  GroundTruth_pose.pose.position.z = tfs.transform.translation.z;
  GroundTruth_pose.pose.orientation.x = tfs.transform.rotation.x;
  GroundTruth_pose.pose.orientation.y = tfs.transform.rotation.y;
  GroundTruth_pose.pose.orientation.z = tfs.transform.rotation.z;
  GroundTruth_pose.pose.orientation.w = tfs.transform.rotation.w;

  vector<float> tfv;
  tfv.push_back(tfs.transform.translation.x);
  tfv.push_back(tfs.transform.translation.y);
  tfv.push_back(tfs.transform.translation.z);
  tfv.push_back(tfs.transform.rotation.x);
  tfv.push_back(tfs.transform.rotation.y);
  tfv.push_back(tfs.transform.rotation.z);
  tfv.push_back(tfs.transform.rotation.w);
  mpSLAM->AddGroundTruth(tfs.header.stamp.toSec(),tfv);
  mvPubCurrentGT = tfv;


  /*---------------GroundTruthPath-------------------*/
  GroundTruthPath.header.stamp = ros::Time::now();
  GroundTruthPath.header.frame_id = "world";
  GroundTruthPath.poses.push_back(GroundTruth_pose);

  LastGroundTruthPath.header.stamp = ros::Time::now();
  LastGroundTruthPath.header.frame_id = "world";

  if (vKeyPose.size()<1) //Reset or initialze
  {
    LastGroundTruthPath.poses.clear();
    // cout << "load last_groundtruth from file"<<endl;
    FILE * pLGTFile = fopen("GroundTruth.csv","r");
    char tmp[10000];
    if(pLGTFile==NULL)
    {
      cerr << "Open GroundTruth.csv failed. Wrong path." << endl;
    }
    else if(fgets(tmp, 10000, pLGTFile) == NULL)
    {
      cerr << "Can't load GroundTruth.csv. File is empty."<< endl;
    }
    else
    {
      double t;
      float px, py, pz, qx, qy, qz, qw;
      while (!feof(pLGTFile))
      {
        if(fscanf(pLGTFile, "%lf,%f,%f,%f,%f,%f,%f,%f", &t,
                &px, &py, &pz, &qx, &qy, &qz, &qw) != EOF)
        {
          geometry_msgs::PoseStamped last_groundtruth;
          last_groundtruth.header.stamp = ros::Time::now();
          last_groundtruth.header.frame_id = "world";
          last_groundtruth.pose.position.x = px;
          last_groundtruth.pose.position.y = py;
          last_groundtruth.pose.position.z = pz+offset;
          last_groundtruth.pose.orientation.x = qx;
          last_groundtruth.pose.orientation.y = qy;
          last_groundtruth.pose.orientation.z = qz;
          last_groundtruth.pose.orientation.w = qw;
          LastGroundTruthPath.poses.push_back(last_groundtruth);
        }
      }
      fclose(pLGTFile);
    }
    GroundTruthPath.poses.clear();
    //preserve the origin of path
    GroundTruth_pose.pose.position.x = mvPubMapOrigin[0];
    GroundTruth_pose.pose.position.y = mvPubMapOrigin[1];
    GroundTruth_pose.pose.position.z = mvPubMapOrigin[2];
    GroundTruth_pose.pose.orientation.x = mvPubMapOrigin[3];
    GroundTruth_pose.pose.orientation.y = mvPubMapOrigin[4];
    GroundTruth_pose.pose.orientation.z = mvPubMapOrigin[5];
    GroundTruth_pose.pose.orientation.w = mvPubMapOrigin[6];
    GroundTruthPath.poses.push_back(GroundTruth_pose);
  }
  lgtpath_.publish(LastGroundTruthPath);
  gtpath_.publish(GroundTruthPath);

  visualization_msgs::Marker similarity_marker;
  similarity_marker.header.stamp = ros::Time::now();
  similarity_marker.header.frame_id = "world";
  similarity_marker.ns = "similarity_marker";
  similarity_marker.id = 0;
  similarity_marker.type = visualization_msgs::Marker::LINE_LIST;
  similarity_marker.action = visualization_msgs::Marker::ADD;
  similarity_marker.scale.x = 0.003;
  similarity_marker.color.r = 0.0f;
  similarity_marker.color.g = 1.0f;
  similarity_marker.color.b = 0.0f;
  similarity_marker.color.a = 1.0;
  vector<vector<float> > vSimilarityMatches = mpSLAM->GetvSimilarityMatches();
  for(vector<vector<float> >::iterator vit=vSimilarityMatches.begin(), vend=vSimilarityMatches.end(); vit!=vend; vit++)
  {
    vector<float> GroundTruth = *vit;
    geometry_msgs::Point p;
    p.x = GroundTruth[0];
    p.y = GroundTruth[1];
    p.z = GroundTruth[2];
    if ((vit-vSimilarityMatches.begin())%2 != 0)//last groundtruth(odd index) is offseted for visualization
      p.z += offset;
    similarity_marker.points.push_back(p);
  }
  sim_.publish(similarity_marker);

}

void ImageGrabber::SetRecord(const std_msgs::Bool& brr)
{
  bool bRequestRecord = brr.data;
  if(mpSLAM->isRecording() && !bRequestRecord)//request stop
    mpSLAM->StopRecord();
  else if(!mpSLAM->isRecording() && bRequestRecord)//request start
    mpSLAM->StartRecord();

}

void ImageGrabber::LoadImgs(const std_msgs::Int8& nr)
{
  if (nr.data == 'p')
  {
    cv::Mat Im1, Im2;
    stringstream name1,name2;
    name1 << "CompImages/"<< nImgsCount << ".png";
    Im1 = cv::imread(name1.str(),CV_LOAD_IMAGE_UNCHANGED);
    nImgsCount++;
    name2 << "CompImages/"<< nImgsCount << ".png";
    Im2 = cv::imread(name2.str(),CV_LOAD_IMAGE_UNCHANGED);
    nImgsCount++;

    if(Im1.empty() || Im2.empty())
    {
      cerr << endl << "Failed to load image at: " <<  name1.str() << endl;
      return;
    }
    if(Im1.channels()==3)
      cvtColor(Im1,Im1,CV_RGB2GRAY);
    if(Im2.channels()==3)
      cvtColor(Im2,Im2,CV_RGB2GRAY);
    mpSLAM->CompareImgs(Im1,Im2);
  }
  cout << "p"<<endl;
}

void ImageGrabber::UpdateNodeHandle(ros::NodeHandle &n)
{
  mpSLAM->UpdateNodeHandle(n);
}
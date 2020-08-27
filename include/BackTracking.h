#ifndef BACKTRACKING_H
#define BACKTRACKING_H

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf2_ros/transform_broadcaster.h>
#include<geometry_msgs/TransformStamped.h>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<unistd.h>
#include<iostream>

#include"Initializer.h"
#include"Tracking.h"
#include"Map.h"
#include"FrameDrawer.h"
#include"Frame.h"
#include"LoadedKeyFrame.h"
#include"ORBVocabulary.h"
#include"LoadedKeyFrameDatabase.h"
#include"Converter.h"

#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class LoadedKeyFrameDatabase;

class BackTracking
{

public:

    // Back Tracking states
    enum eBackTrackingState{
        NOT_INITIALIZED=1,
        OK=2
    };

    BackTracking(ORBVocabulary* pVoc, LoadedKeyFrameDatabase* pLKFDB, Tracking* pTracker, FrameDrawer* pFrameDrawer, unsigned int nKFload, bool bDBload, const string &strSettingPath);

    void Run();

    void Update();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();//release isStopped flag, continue back track on new-arrived frame

    bool isBackTrack();

    bool isOnCommand();

    void Activate(Map* pMap,KeyFrameDatabase* pKFDB);

private:
    bool Stop();

    bool CheckFinish();

    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool CheckNewFrames();

    long unsigned int BackTrack(Frame* mpCurrentFrame,ofstream& BTlog);

protected:
    //TRUE: if nLKF!=0, bDBload==1 and Backtracking.Setting=1 or 2 in yaml file.
    bool mbBackTrack;
    bool mbForward;//default backward
    bool mbOnCommand;

    std::mutex mMutexBackTrack;//for mbBackTrack and mbForward

    eBackTrackingState mState;
    // Frame* mpCurrentFrame;
    list<Frame> mlFrameBuffer;
    std::mutex mMutexBuffer;

    long unsigned int mnCurrentLKFId;
    LoadedKeyFrame* mpNextLKF;

    //Tracker for Current Frame
    Tracking* mpTracker;

    //Initializer for pose estimate
    Initializer* mpInitializer;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;

    //FrameDrawer
    FrameDrawer* mpFrameDrawer;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    LoadedKeyFrameDatabase* mpLoadedKeyFrameDB;

    //SearchByBoW
    std::vector<int> mvBoWMatches;


    //Publisher
    cv::Mat mTwv = (cv::Mat_<float>(4,4) << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1);//from c1 to c2
    cv::Mat mTbc = (cv::Mat_<float>(4,4) << 
      0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
        0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
      -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
        0.0, 0.0, 0.0, 1.0);
    cv::Mat mTbg = (cv::Mat_<float>(4,4) << 
          0.33638, -0.01749,  0.94156,  0.06901,
          -0.02078, -0.99972, -0.01114, -0.02781,
          0.94150, -0.01582, -0.33665, -0.12395,
              0.0,      0.0,      0.0,      1.0);
    cv::Mat mTcb = (cv::Mat_<float>(4,4) << 
    0.01486554,   	0.99955725,   	-0.02577444,   	0.06522291,   
    -0.99988093,   	0.01496721,   	0.00375619,   	-0.02070639,   
    0.00414030,   	0.02571553,   	0.99966073,   	-0.00805460,   
    0.00000000,   	0.00000000,   	0.00000000,   	1.00000000);
    cv::Mat mTgb = (cv::Mat_<float>(4,4) << 
    0.33638045,   	-0.02078355,   	0.94149385,   	0.09290656,   
    -0.01748395,   	-0.99972388,   	-0.01581839,   	-0.02855644,   
    0.94156777,   	-0.01114533,   	-0.33665020,   	-0.10701534,   
    0.00000000,   	0.00000000,   	0.00000000,   	1.00000000);

    ros::Publisher Pub_Tcr;
    tf2_ros::TransformBroadcaster Pub_tf2;
};

} //namespace ORB_SLAM

#endif // BACKTRACKING_H

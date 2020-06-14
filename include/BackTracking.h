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

class BackTracking
{

public:

    // Back Tracking states
    enum eBackTrackingState{
        NOT_INITIALIZED=1,
        OK=2
    };

    BackTracking(ORBVocabulary* pVoc, LoadedKeyFrameDatabase* pLKFDBm, Tracking* pTracker, FrameDrawer* pFrameDrawer, unsigned int nKFload, bool bDBload, const string &strSettingPath);

    void Run();

    void Update();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();//release isStopped flag, continue back track on new-arrived frame

    bool isBackTrack();

    void RegisterNodeHandle(ros::NodeHandle &n);

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
    //TRUE: if nLKF!=0, bDBload==1 and Backtracking.Setting!=0 in yaml file.
    bool mbBackTrack;
    bool mbForward;

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
    ros::Publisher Pub_Tcr;
    tf2_ros::TransformBroadcaster Pub_tf2;
};

} //namespace ORB_SLAM

#endif // BACKTRACKING_H

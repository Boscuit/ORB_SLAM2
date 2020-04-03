#ifndef BACKTRACKING_H
#define BACKTRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<unistd.h>

#include"Initializer.h"
#include"Tracking.h"
#include"FrameDrawer.h"
#include"Frame.h"
#include"LoadedKeyFrame.h"
#include"ORBVocabulary.h"
#include"LoadedKeyFrameDatabase.h"

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

    void Update(Tracking *pTracker);

    void RequestFinish();

    bool isFinished();

    bool isBackTrack();

    // cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

private:

    bool CheckFinish();

    void SetFinish();

    bool CheckNewFrames();

    long unsigned int BackTrack(Frame* mpCurrentFrame);


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

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

};

} //namespace ORB_SLAM

#endif // BACKTRACKING_H

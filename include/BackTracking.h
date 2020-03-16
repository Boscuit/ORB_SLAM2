#ifndef BACKTRACKING_H
#define BACKTRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<unistd.h>

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
        READY=0,
        ARRIVE=1,
        PROCESSING=2
    };

    BackTracking(ORBVocabulary* pVoc, LoadedKeyFrameDatabase* pLKFDBm, Tracking* pTracker, FrameDrawer* pFrameDrawer);

    void Run();

    void Update(Tracking *pTracker);

    long unsigned int BackTrack(Frame* mpCurrentFrame);

    void RequestFinish();

    bool isFinished();

    // cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

public:

    int mnCandidate;

protected:

    eBackTrackingState mState;
    // Frame* mpCurrentFrame;
    Frame mCurrentFrame;
    std::mutex mMutex;

    long unsigned int mnCurrentId;
    long unsigned int mnNextId;

    //Tracker for Current Frame
    Tracking* mpTracker;

    //FrameDrawer
    FrameDrawer* mpFrameDrawer;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    LoadedKeyFrameDatabase* mpLoadedKeyFrameDB;


    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

};

} //namespace ORB_SLAM

#endif // BACKTRACKING_H

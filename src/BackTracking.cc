#include"BackTracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Tracking.h"
#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

BackTracking::BackTracking(ORBVocabulary* pVoc, LoadedKeyFrameDatabase* pLKFDB, Tracking* pTracker, FrameDrawer* pFrameDrawer):
    mState(READY), mnCandidate(0), mnCurrentId(0), mnNextId(0), mbFinishRequested(false), mbFinished(true),
    mpORBVocabulary(pVoc), mpLoadedKeyFrameDB(pLKFDB), mpTracker(pTracker),mpFrameDrawer(pFrameDrawer)
{
}


void BackTracking::Run()
{
  mbFinished = false;
  int state;
  while(1)
  {
    {
      unique_lock<mutex> lock(mMutex);
      state = mState;
      if(mState == ARRIVE)
        mState = PROCESSING;
    }
    if (state == ARRIVE)
    {
      // cout <<mCurrentFrame.mnId<<endl;
      long unsigned int result = BackTrack(&mCurrentFrame);
      // cout<<"Bset match: " << result <<endl;
      {
        unique_lock<mutex> lock(mMutex);
        mState = READY;
      }

    }

    if(CheckFinish())
      break;

    usleep(3000);
  }

  SetFinish();
}


void BackTracking::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool BackTracking::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void BackTracking::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool BackTracking::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void BackTracking::Update(Tracking *pTracker)
{
  unique_lock<mutex> lock(mMutex);
  if (pTracker->mState == Tracking::OK && mState == READY)
  {
    mState = ARRIVE;
    mCurrentFrame = mpTracker->mCurrentFrame;
    // cout<<"BackTrack updated! ";
  }
}

long unsigned int BackTracking::BackTrack(Frame* mpCurrentFrame)
{
    mnCandidate = 0;
    float scoreTH = 0.9;
    // Compute Bag of Words Vector
    mpCurrentFrame->ComputeBoW();

    cout<<"CF: "<<mpCurrentFrame->mnId<<" BT... "<<endl;

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation

    //sorted by BoW score in descending order
    vector<LoadedKeyFrame*> vpCandidateLKFs = mpLoadedKeyFrameDB->DetectBackTrackCandidates(mpCurrentFrame,5);
    map<long unsigned int, float> mvSimilarity = mpLoadedKeyFrameDB->GetSimilarity();
    mpFrameDrawer->UpdateSimilarity(mvSimilarity,mpCurrentFrame->mnId);

    if(vpCandidateLKFs.empty())
        return 0;

    LoadedKeyFrame* pBestLKF = vpCandidateLKFs[0];
    if(pBestLKF->mBackTrackScore>scoreTH)
    {
      unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
      map<double,vector<float>>::iterator it = mpTracker->mvGroundTruth.lower_bound(mpCurrentFrame->mTimeStamp);
      if(it==mpTracker->mvGroundTruth.end())
        it--;
      vector<float> currentFrameGT = it->second;
      vector<float> bestLKFGT = pBestLKF->mGroundTruth;
      mpTracker->mlSimilarityMatches.push_back(currentFrameGT);
      mpTracker->mlSimilarityMatches.push_back(bestLKFGT);
    }

    const int nLKFs = vpCandidateLKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    // vector<PnPsolver*> vpPnPsolvers;
    // vpPnPsolvers.resize(nLKFs);
    //
    // vector<vector<MapPoint*> > vvpMapPointMatches;
    // vvpMapPointMatches.resize(nLKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nLKFs);
    // LoadedKeyFrame* pLKF=vpCandidateLKFs[0];

    vector<LoadedKeyFrame*> vpOutstandingLKFs;

    for(int i=0; i<nLKFs; i++)
    {
        LoadedKeyFrame* pLKF = vpCandidateLKFs[i];
        // if(pKF->isBad())
        //     vbDiscarded[i] = true;
        // else
        // {
        int nmatches = matcher.SearchByBoW(pLKF,*mpCurrentFrame);
        if(nmatches<15)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            vpOutstandingLKFs.push_back(pLKF);
            // PnPsolver* pSolver = new PnPsolver(*mpCurrentFrame,vvpMapPointMatches[i]);
            // pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
            // vpPnPsolvers[i] = pSolver;
            mnCandidate++;
        }
        // }
    }

    cout << "match: ";
    for(int can=0; can<mnCandidate; can++)
    {
      cout << vpOutstandingLKFs[can]->mnId << " ";
    }
    cout << endl;

    if(!mnCurrentId && mnCandidate)//means mnCurrentId==0 and mnCandidate!=0
    {
      mnCurrentId = vpOutstandingLKFs[0]->mnId;//initialze with the first detected frame id
      mnNextId = mnCurrentId+1;
    }

    // for(int can=0; can<mnCandidate; can++)
    // {
    //   if(vpOutstandingLKFs[can]->mnId == mnNextId)//position is reached
    //   {
    //     mnCurrentId = mnNextId++;
    //     break;
    //   }
    // }

    if(mnCandidate==1)
    {
      if(vpOutstandingLKFs[0]->mnId == mnNextId)
        mnCurrentId = mnNextId++;
    }
    // cout << "Current: " << mnCurrentId << ", Next: " << mnNextId << "." << endl;

    return mnNextId;

    // // Alternatively perform some iterations of P4P RANSAC
    // // Until we found a camera pose supported by enough inliers
    // bool bMatch = false;
    // ORBmatcher matcher2(0.9,true);
    //
    // while(nCandidates>0 && !bMatch)
    // {
    //     for(int i=0; i<nLKFs; i++)
    //     {
    //         if(vbDiscarded[i])
    //             continue;
    //
    //         // Perform 5 Ransac Iterations
    //         vector<bool> vbInliers;
    //         int nInliers;
    //         bool bNoMore;
    //
    //         PnPsolver* pSolver = vpPnPsolvers[i];
    //         cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
    //
    //         // If Ransac reachs max. iterations discard keyframe
    //         if(bNoMore)
    //         {
    //             vbDiscarded[i]=true;
    //             nCandidates--;
    //         }
    //
    //         // If a Camera Pose is computed, optimize
    //         if(!Tcw.empty())
    //         {
    //             Tcw.copyTo(mpCurrentFrame->mTcw);
    //
    //             set<MapPoint*> sFound;
    //
    //             const int np = vbInliers.size();
    //
    //             for(int j=0; j<np; j++)
    //             {
    //                 if(vbInliers[j])
    //                 {
    //                     mpCurrentFrame->mvpMapPoints[j]=vvpMapPointMatches[i][j];
    //                     sFound.insert(vvpMapPointMatches[i][j]);
    //                 }
    //                 else
    //                     mpCurrentFrame->mvpMapPoints[j]=NULL;
    //             }
    //
    //             int nGood = Optimizer::PoseOptimization(mpCurrentFrame);
    //
    //             if(nGood<10)
    //                 continue;
    //
    //             for(int io =0; io<mpCurrentFrame->N; io++)
    //                 if(mpCurrentFrame->mvbOutlier[io])
    //                     mpCurrentFrame->mvpMapPoints[io]=static_cast<MapPoint*>(NULL);
    //
    //             // If few inliers, search by projection in a coarse window and optimize again
    //             if(nGood<50)
    //             {
    //                 int nadditional =matcher2.SearchByProjection(*mpCurrentFrame,vpCandidateLKFs[i],sFound,10,100);
    //
    //                 if(nadditional+nGood>=50)
    //                 {
    //                     nGood = Optimizer::PoseOptimization(mpCurrentFrame);
    //
    //                     // If many inliers but still not enough, search by projection again in a narrower window
    //                     // the camera has been already optimized with many points
    //                     if(nGood>30 && nGood<50)
    //                     {
    //                         sFound.clear();
    //                         for(int ip =0; ip<mpCurrentFrame->N; ip++)
    //                             if(mpCurrentFrame->mvpMapPoints[ip])
    //                                 sFound.insert(mpCurrentFrame->mvpMapPoints[ip]);
    //                         nadditional =matcher2.SearchByProjection(*mpCurrentFrame,vpCandidateLKFs[i],sFound,3,64);
    //
    //                         // Final optimization
    //                         if(nGood+nadditional>=50)
    //                         {
    //                             nGood = Optimizer::PoseOptimization(mpCurrentFrame);
    //
    //                             for(int io =0; io<mpCurrentFrame->N; io++)
    //                                 if(mpCurrentFrame->mvbOutlier[io])
    //                                     mpCurrentFrame->mvpMapPoints[io]=NULL;
    //                         }
    //                     }
    //                 }
    //             }
    //
    //
    //             // If the pose is supported by enough inliers stop ransacs and continue
    //             if(nGood>=50)
    //             {
    //                 bMatch = true;
    //                 break;
    //             }
    //         }
    //     }
    // }
    //
    // if(!bMatch)
    // {
    //     return false;
    // }
    // else
    // {
    //     mnLastRelocFrameId = mpCurrentFrame->mnId;
    //     return true;
    // }


}



} //namespace ORB_SLAM

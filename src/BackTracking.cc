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
    mState(NOT_INITIALIZED), mnCurrentLKFId(0), mpNextLKF(static_cast<LoadedKeyFrame*>(NULL)), mbFinishRequested(false), mbFinished(true),
    mpInitializer(static_cast<Initializer*>(NULL)), mpORBVocabulary(pVoc), mpLoadedKeyFrameDB(pLKFDB), mpTracker(pTracker),mpFrameDrawer(pFrameDrawer)
{
}


void BackTracking::Run()
{
  mbFinished = false;
  while(1)
  {
    if (CheckNewFrames())
    {
      // cout <<mCurrentFrame.mnId<<endl;
      Frame CurrentFrame;
      {
        unique_lock<mutex> lock(mMutexBuffer);
        cout << "Buffer size: " << mlFrameBuffer.size();
        CurrentFrame = mlFrameBuffer.front();
        mlFrameBuffer.pop_front();
      }
      long unsigned int result = BackTrack(&CurrentFrame);
      // cout<<"Bset match: " << result <<endl;
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
  if (pTracker->mState == Tracking::OK)
  {
    unique_lock<mutex> lock(mMutexBuffer);
    if (mlFrameBuffer.size()<5)
      mlFrameBuffer.push_back(Frame(mpTracker->mCurrentFrame));
    // cout<<"Buffer size: " << mlFrameBuffer.size() << ".  ";
  }
}

bool BackTracking::CheckNewFrames()
{
    unique_lock<mutex> lock(mMutexBuffer);
    return(!mlFrameBuffer.empty());
}

long unsigned int BackTracking::BackTrack(Frame* mpCurrentFrame)
{
    bool bForward = true;
    float matchTH = 0.1;
    float lostTH = 0.01;
    // Compute Bag of Words Vector
    mpCurrentFrame->ComputeBoW();

    // Obtain groundtruth of current frames
    map<double,vector<float>>::iterator it = mpTracker->mvGroundTruth.lower_bound(mpCurrentFrame->mTimeStamp);
    if(it==mpTracker->mvGroundTruth.end())
      it--;
    vector<float> currentFrameGT = it->second;

    cout<<" CF: "<<mpCurrentFrame->mnId<<" BT... ";


    //Obtain BoW score and sorted vector by BoW score in descending order
    vector<LoadedKeyFrame*> vpCandidateLKFs = mpLoadedKeyFrameDB->DetectBackTrackCandidates(mpCurrentFrame,5);
    map<long unsigned int, float> mvSimilarity = mpLoadedKeyFrameDB->GetSimilarity();
    mpFrameDrawer->UpdateSimilarity(mvSimilarity,mpCurrentFrame->mnId);

    if(vpCandidateLKFs.empty())
        return 0;

    LoadedKeyFrame* pBestLKF = vpCandidateLKFs[0];
    if(mState == NOT_INITIALIZED)
    {
      // mpNextLKF = pBestLKF;
      // if(mpNextLKF->mBackTrackScore>matchTH)
      if(pBestLKF->mBackTrackScore>matchTH)//
      {
        cout << "Initial matched"<<endl;
        mState = OK;//BackTrack is initialized only when a match happened
        unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
        mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
        // mpTracker->mvSimilarityMatches.push_back(mpNextLKF->mGroundTruth);
        // mnCurrentLKFId = mpNextLKF->mnId;
        mpTracker->mvSimilarityMatches.push_back(pBestLKF->mGroundTruth);//
        mnCurrentLKFId = pBestLKF->mnId;//
        mpNextLKF = mpLoadedKeyFrameDB->GetNextLKF(mnCurrentLKFId,bForward);
      }
    }
    else//mState == OK
    {
      unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
      mpTracker->mvSimilarityMatches.pop_back();
      mpTracker->mvSimilarityMatches.pop_back();
      if(mpNextLKF->mBackTrackScore>matchTH)
      {
        cout << "matched. score: "<<mpNextLKF->mBackTrackScore;
        mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
        mpTracker->mvSimilarityMatches.push_back(mpNextLKF->mGroundTruth);
        mnCurrentLKFId = mpNextLKF->mnId;
        mpNextLKF = mpLoadedKeyFrameDB->GetNextLKF(mnCurrentLKFId,bForward);
        cout << " CurrentLKF: "<< mnCurrentLKFId<<". NextLKF: "<<mpNextLKF->mnId<<". "<<endl;
      }
      else if(mpNextLKF->mBackTrackScore<lostTH)//relocalization
      {
        cout << "lost. score: "<<mpNextLKF->mBackTrackScore;
        mpNextLKF = pBestLKF;
        cout << " CurrentLKF: "<< mnCurrentLKFId<<". NextLKF: "<<mpNextLKF->mnId<<". Reloc score: "<<mpNextLKF->mBackTrackScore<< "."<<endl;
      }
      else
        cout << "Approching. CurrentLKF: "<< mnCurrentLKFId<<". NextLKF: "<<mpNextLKF->mnId<<". Score: "<<mpNextLKF->mBackTrackScore<< "."<<endl;
    }
    if(mpLoadedKeyFrameDB->IsLast(mnCurrentLKFId, bForward))
    {
      cout << "Reach the end."<<endl;
      RequestFinish();
      return 0;
    }

    if(mpNextLKF != NULL)
    {
      unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
      mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
      mpTracker->mvSimilarityMatches.push_back(mpNextLKF->mGroundTruth);

      //Calculate R and t from current frame to NextLKF
      if(mpInitializer != NULL)
      {
        // cout <<"delete initializer"<<endl;
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
      }
      // cout << "New initializer"<<endl;
      mpInitializer =  new Initializer(mpNextLKF->mvKeys,mpCurrentFrame->mK,0.1,200);//mK is Calibration matrix
      fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

      //If user have matches between two points set (mvIniMatches), don't need to search
      //------------------------------------------------------------
      // cout<< "mvbPrevMatched initialize"<<endl;
      mvbPrevMatched.resize(mpNextLKF->mvKeys.size());
      for(size_t i=0; i<mpNextLKF->mvKeys.size(); i++)
          mvbPrevMatched[i]=mpNextLKF->mvKeys[i].pt;

      // Find correspondences
      ORBmatcher matcher(0.9,true);
      // cout<<"serchbyinitialization"<<endl;
      int nInimatches = matcher.SearchForInitialization(mpNextLKF,*mpCurrentFrame,mvbPrevMatched,mvIniMatches,100);
      // Check if there are enough correspondences
      // cout<<"serchbyinitialization done. nInimatches: "<< nInimatches << endl;
      if(nInimatches<10)
      {
        cout << "Not enough correspondences! Pose estimate may be not accurate." << endl;
        return 0;
      }
      //------------------------------------------------------------

      cv::Mat Rcw(3,3,CV_32F); // Current Camera Rotation
      cv::Mat tcw(3,1,CV_32F); // Current Camera Translation
      vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
      // cout<<"initialize"<<endl;
      if(mpInitializer->Initialize(mpCurrentFrame->mvKeysUn, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
      {
        cout<<"initialize succeed"<<endl;
          for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
          {
              if(mvIniMatches[i]>=0 && !vbTriangulated[i])
              {
                mvIniMatches[i]=-1;
                nInimatches--;
              }
          }

          // Set Frame Poses
          cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
          Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
          tcw.copyTo(Tcw.rowRange(0,3).col(3));
          cout << Tcw << endl;
      }
    }

    // if(pBestLKF->mBackTrackScore>matchTH)
    // {
    //   mnCurrentLKFId = pBestLKF->mnId;
    //   unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
    //   map<double,vector<float>>::iterator it = mpTracker->mvGroundTruth.lower_bound(mpCurrentFrame->mTimeStamp);
    //   if(it==mpTracker->mvGroundTruth.end())
    //     it--;
    //   vector<float> currentFrameGT = it->second;
    //   vector<float> bestLKFGT = pBestLKF->mGroundTruth;
    //   if(mnLastLKFId != 0)//remove link from last frame before adding a new link of matches
    //   {
    //     // mpTracker->mvSimilarityMatches.pop_back();
    //     // mpTracker->mvSimilarityMatches.pop_back();
    //   }
    //   mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
    //   mpTracker->mvSimilarityMatches.push_back(bestLKFGT);
    //   mpNextLKF = mpLoadedKeyFrameDB->GetNextLKF(mnCurrentLKFId,bForward);
    //
    //   if(mpInitializer != NULL)
    //   {
    //     // cout <<"delete initializer"<<endl;
    //     delete mpInitializer;
    //     mpInitializer = static_cast<Initializer*>(NULL);
    //   }
    //   // cout << "New initializer"<<endl;
    //   mpInitializer =  new Initializer(mpNextLKF->mvKeys,mpCurrentFrame->mK,0.1,200);//mK is Calibration matrix
    //   fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
    //
    //   //If user have matches between two points set (mvIniMatches), don't need to search
    //   //------------------------------------------------------------
    //   // cout<< "mvbPrevMatched initialize"<<endl;
    //   mvbPrevMatched.resize(mpNextLKF->mvKeys.size());
    //   for(size_t i=0; i<mpNextLKF->mvKeys.size(); i++)
    //       mvbPrevMatched[i]=mpNextLKF->mvKeys[i].pt;
    //
    //   // Find correspondences
    //   ORBmatcher matcher(0.9,true);
    //   // cout<<"serchbyinitialization"<<endl;
    //   int nInimatches = matcher.SearchForInitialization(mpNextLKF,*mpCurrentFrame,mvbPrevMatched,mvIniMatches,100);
    //   // Check if there are enough correspondences
    //   // cout<<"serchbyinitialization done. nInimatches: "<< nInimatches << endl;
    //   if(nInimatches<10)
    //   {
    //     cout << "Not enough correspondences! Pose estimate may be not accurate." << endl;
    //     return 0;
    //   }
    //   //------------------------------------------------------------
    //
    //   cv::Mat Rcw(3,3,CV_32F); // Current Camera Rotation
    //   cv::Mat tcw(3,1,CV_32F); // Current Camera Translation
    //   vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    //   // cout<<"initialize"<<endl;
    //   if(mpInitializer->Initialize(mpCurrentFrame->mvKeysUn, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
    //   {
    //     cout<<"initialize succeed"<<endl;
    //       for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
    //       {
    //           if(mvIniMatches[i]>=0 && !vbTriangulated[i])
    //           {
    //             mvIniMatches[i]=-1;
    //             nInimatches--;
    //           }
    //       }
    //
    //       // Set Frame Poses
    //       cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    //       Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    //       tcw.copyTo(Tcw.rowRange(0,3).col(3));
    //       cout << Tcw << endl;
    //   }
    // }
    // else if(mnCurrentLKFId != 0)
    // {
    //   unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
    //   map<double,vector<float>>::iterator it = mpTracker->mvGroundTruth.lower_bound(mpCurrentFrame->mTimeStamp);
    //   if(it==mpTracker->mvGroundTruth.end())
    //     it--;
    //
    //   vector<float> currentFrameGT = it->second;
    //   vector<float> nextLKFGT = mpNextLKF->mGroundTruth;
    //   if (mnLastLKFId == mnCurrentLKFId)
    //   {
    //     // mpTracker->mvSimilarityMatches.pop_back();
    //     // mpTracker->mvSimilarityMatches.pop_back();
    //   }
    //   else
    //     mnLastLKFId = mnCurrentLKFId;
    //   mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
    //   mpTracker->mvSimilarityMatches.push_back(nextLKFGT);
    // }

    // ORBmatcher matcher(0.75,true);
    // int nmatches = matcher.SearchByBoW(mpNextLKF,*mpCurrentFrame);
    return mnCurrentLKFId;

}



} //namespace ORB_SLAM

#include"BackTracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Tracking.h"
#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

BackTracking::BackTracking(ORBVocabulary* pVoc, LoadedKeyFrameDatabase* pLKFDB, Tracking* pTracker, FrameDrawer* pFrameDrawer, unsigned int nKFload, bool bDBload, const string &strSettingPath):
    mState(NOT_INITIALIZED), mnCurrentLKFId(0), mpNextLKF(static_cast<LoadedKeyFrame*>(NULL)), mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false), mbBackTrack(false), mbForward(false), mbOnCommand(false),
    mpInitializer(static_cast<Initializer*>(NULL)), mpORBVocabulary(pVoc), mpLoadedKeyFrameDB(pLKFDB), mpTracker(pTracker),mpFrameDrawer(pFrameDrawer)
{
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  int BackTrackMode = fSettings["BackTracking.Mode"];
  cout << "BackTrack.Mode: " << BackTrackMode << endl;
  bool bLoadSucceed = (nKFload!=0) && bDBload;
  pFrameDrawer->setSimilarity(max(nKFload,(unsigned int)1),1000);

  mnCurrentLKFId = 0;
  mpNextLKF = static_cast<LoadedKeyFrame*>(NULL);

  switch (BackTrackMode){
  case 1:
    mbBackTrack = bLoadSucceed;
    mbForward = true;
    break;
  case 2:
    mbBackTrack = bLoadSucceed;
    mbForward = false;
    break;
  case 3://mbBackTrack:false
    mbOnCommand = true;
    break;
  default://include 0
    break;
  }
}


void BackTracking::Run()
{
  mbFinished = false;//if BackTracking thread is not open. mbFinished is default as true.
  mbStopped = false;
  
  ofstream BTlog;
  BTlog.open("BTlog.txt");
  BTlog << fixed;
  while(1)
  {
    if (CheckNewFrames())
    {
      // cout <<mCurrentFrame.mnId<<endl;
      Frame CurrentFrame;
      {
        unique_lock<mutex> lock(mMutexBuffer);
        //cout << "mMutextBuffer locked by CurrentFrame."<<endl;
        cout << "Buffer size: " << mlFrameBuffer.size();
        CurrentFrame = mlFrameBuffer.front();
        mlFrameBuffer.pop_front();
      }
      //cout << "mMutextBuffer free."<<endl;
      //cout << "CurrentFrame: "<<CurrentFrame.mnId<<endl;
      long unsigned int nMatches = BackTrack(&CurrentFrame,BTlog);
      // cout<<"Bset match: " << result <<endl;
    }

    if(Stop())
    {
      {
        unique_lock<mutex> lock(mMutexBuffer);
        unique_lock<mutex> lock2(mMutexBackTrack);
        mlFrameBuffer.clear();//only lock when clearing
        mbBackTrackPrev = mbBackTrack;
        cout << "Store mbBackTrackPrev: " << mbBackTrack << "->" << mbBackTrackPrev << endl;
        mbBackTrack = false;
      }
      while(isStopped())
      {
        usleep(3000);
        if(CheckFinish())
          break;
      }
    }

    if(CheckFinish())
      break;

    usleep(3000);
  }
  cout << "Request Finish" << endl;
  unique_lock<mutex> lock2(mMutexBackTrack);
  mbBackTrack = false;//when finish
  BTlog.close();
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

void BackTracking::RequestStop()
{
  cout << "BackTrack Request Stop" << endl;
  unique_lock<mutex> lock(mMutexStop);
  if(!mbStopped)
      mbStopRequested = true;
}

bool BackTracking::isStopped()
{
  //cout << "isStopped: " << mbStopped << endl;
  unique_lock<mutex> lock(mMutexStop);
  return mbStopped;
}

bool BackTracking::Stop()
{
  //cout << "Stop" << endl;
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        cout << "BT stop" << endl;
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }
    return false;
}

void BackTracking::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexBackTrack);
    cout << "BT Release" << endl;
    mbStopped = false;
    mbBackTrack = mbBackTrackPrev;
    cout << "Restore mbBackTrack: " << mbBackTrackPrev << "->" << mbBackTrack << endl;
    
}

bool BackTracking::isBackTrack()
{
    unique_lock<mutex> lock(mMutexBackTrack);
    cout << "isBackTrack:" << mbBackTrack << endl;
    return mbBackTrack;
}

bool BackTracking::isOnCommand()
{
    return mbOnCommand;
}

void BackTracking::Activate(Map* pMap,KeyFrameDatabase* pKFDB)
{
  {
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if (mbFinishRequested)
      return;
    if (mbStopped)//every time activate after request stop, but not the first activation after first start/reset(mbStopped=false)
      mbStopped = false;
  }
  //only active when mbBacktrack is false
  unsigned int nKFload = mpLoadedKeyFrameDB->LoadLKFFromMap(pMap,mpTracker);
  bool bDBload = mpLoadedKeyFrameDB->LoadDBFromKFDB(pKFDB);
  if ((nKFload!=0) && bDBload)
  {
    unique_lock<mutex> lock(mMutexBackTrack);
    mbBackTrack = true;
    cout << "BT activated." << endl;
  }
  else
  {
    cout << "Load online trajectory failed." << endl;
  }
}

void BackTracking::Update()
{
  //frame may be deleted if wrong initialization, will left a invaild
  if (mpTracker->mState == Tracking::OK)
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

void BackTracking::RegisterNodeHandle(ros::NodeHandle &n)
{
  Pub_Tcr = n.advertise<geometry_msgs::PoseStamped>("Tcr_pose",1);
}

long unsigned int BackTracking::BackTrack(Frame* mpCurrentFrame,ofstream& BTlog)
{
    float matchTH = 0.1;
    float lostTH = 0.01;
    int nInimatches;//Number of matches in SearchForInitialization
    int nBoWmatches;//Number of matches in SearchByBoW

    // Compute Bag of Words Vector
    mpCurrentFrame->ComputeBoW();

    // Obtain groundtruth of current frames
    map<double,vector<float>>::iterator it = mpTracker->mvGroundTruth.lower_bound(mpCurrentFrame->mTimeStamp);
    if(it==mpTracker->mvGroundTruth.end())
      it--;
    vector<float> currentFrameGT = it->second;

    //Publish Current Groundtruth
    geometry_msgs::TransformStamped current_gt;
    current_gt.header.stamp = ros::Time::now();
    current_gt.header.frame_id = "world";
    current_gt.child_frame_id = "Current_Groundtruth";
    current_gt.transform.translation.x = currentFrameGT[0];
    current_gt.transform.translation.y = currentFrameGT[1];
    current_gt.transform.translation.z = currentFrameGT[2];
    current_gt.transform.rotation.x = currentFrameGT[3];
    current_gt.transform.rotation.y = currentFrameGT[4];
    current_gt.transform.rotation.z = currentFrameGT[5];
    current_gt.transform.rotation.w = currentFrameGT[6];
    Pub_tf2.sendTransform(current_gt);

    cout<<" CF: "<<mpCurrentFrame->mnId<<" BT... ";


    //Obtain BoW score and sorted vector by BoW score in descending order
    vector<LoadedKeyFrame*> vpCandidateLKFs = mpLoadedKeyFrameDB->DetectBackTrackCandidates(mpCurrentFrame,5);
    map<long unsigned int, float> mvSimilarity = mpLoadedKeyFrameDB->GetSimilarity();
    mpFrameDrawer->UpdateSimilarity(mvSimilarity,mpCurrentFrame->mnId);

    if(vpCandidateLKFs.empty())
        return 0;

    LoadedKeyFrame* pBestLKF = vpCandidateLKFs[0];
    LoadedKeyFrame** ppDsrLKF = &mpNextLKF;
    if(mState == NOT_INITIALIZED)
    {
      mpNextLKF = pBestLKF;
      // if(mpNextLKF->mBackTrackScore>matchTH)
      mnCurrentLKFId = pBestLKF->mnId;
      cout << "mnCurrentLKFId: " << mnCurrentLKFId << endl;
      if(pBestLKF->mBackTrackScore>matchTH)//
      {
        cout << "Initial matched"<<endl;
        mState = OK;//BackTrack is initialized only when a match happened
        unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
        mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
        mpTracker->mvSimilarityMatches.push_back(pBestLKF->mGroundTruth);//
        mpNextLKF = mpLoadedKeyFrameDB->GetNextLKF(mnCurrentLKFId,mbForward);
      }
    }
    else//mState == OK
    {
      unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
      mpTracker->mvSimilarityMatches.pop_back();
      mpTracker->mvSimilarityMatches.pop_back();
      if((*ppDsrLKF)->mBackTrackScore>matchTH)
      {
        cout << "matched. score: " << (*ppDsrLKF)->mBackTrackScore;
        mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
        mpTracker->mvSimilarityMatches.push_back((*ppDsrLKF)->mGroundTruth);
        mnCurrentLKFId = (*ppDsrLKF)->mnId;
        mpNextLKF = mpLoadedKeyFrameDB->GetNextLKF(mnCurrentLKFId,mbForward);
        //pDsrLKF = pBestLKF;//updata need to be match pre setting
        cout << " LKF "<< mnCurrentLKFId<<" hit. --> "<<(*ppDsrLKF)->mnId<<". "<<endl;
      }
      else if((*ppDsrLKF)->mBackTrackScore<lostTH)//relocalization
      {
        cout << "lost. score: " << (*ppDsrLKF)->mBackTrackScore;
        mnCurrentLKFId = pBestLKF->mnId;
        mpNextLKF = mpLoadedKeyFrameDB->GetNextLKF(mnCurrentLKFId,mbForward);
        //pDsrLKF = pBestLKF;//updata need to be match pre setting
        cout << " LKF "<< mnCurrentLKFId<<" lost. --> "<<(*ppDsrLKF)->mnId<<". Reloc score: "<<(*ppDsrLKF)->mBackTrackScore<< "."<<endl;
      }
      else
        cout << " LKF "<< mnCurrentLKFId<<" --> "<<(*ppDsrLKF)->mnId<<". Score: "<<(*ppDsrLKF)->mBackTrackScore<< "."<<endl;
    }
    if(mpLoadedKeyFrameDB->IsLast(mnCurrentLKFId, mbForward))
    {
      cout << "Reach the end."<<endl;
      // vector<cv::KeyPoint> vEmptyKeys;
      // vector<int> vEmptyMatches;
      // mpFrameDrawer -> UpdateBTMatch(vEmptyKeys ,vEmptyMatches);//clear the ref KeyPoints in FrameDrawer
      RequestStop();
      return 0;
    }

    if((*ppDsrLKF) != NULL)
    {
      //Publish Desired Groundtruth
      vector <float> desiredLKFGT = (*ppDsrLKF)->mGroundTruth;
      geometry_msgs::TransformStamped desired_gt;
      desired_gt.header.stamp = ros::Time::now();
      desired_gt.header.frame_id = "world";
      desired_gt.child_frame_id = "Desired_Groundtruth";
      desired_gt.transform.translation.x = desiredLKFGT[0];
      desired_gt.transform.translation.y = desiredLKFGT[1];
      desired_gt.transform.translation.z = desiredLKFGT[2];
      desired_gt.transform.rotation.x = desiredLKFGT[3];
      desired_gt.transform.rotation.y = desiredLKFGT[4];
      desired_gt.transform.rotation.z = desiredLKFGT[5];
      desired_gt.transform.rotation.w = desiredLKFGT[6];
      Pub_tf2.sendTransform(desired_gt);


      unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
      mpTracker->mvSimilarityMatches.push_back(currentFrameGT);
      mpTracker->mvSimilarityMatches.push_back((*ppDsrLKF)->mGroundTruth);

      ORBmatcher matcher2(0.75,true);
      vector<int> vBTMatches21;
      nBoWmatches = matcher2.SearchByBoW((*ppDsrLKF),*mpCurrentFrame,vBTMatches21);
      cout<<"SbBoW 0.75 matches: "<< nBoWmatches << endl;
      BTlog<<"Current Frame: "<<mpCurrentFrame->mnId<<", desired LKF: "<<(*ppDsrLKF)->mnId<<", SbBoW 0.75 matches: "<< nBoWmatches << endl;
      mpFrameDrawer -> UpdateBTMatch((*ppDsrLKF)->mvKeys,vBTMatches21,mpLoadedKeyFrameDB->mvLoadedImages[(*ppDsrLKF)->mnId]);

      vector<int> vBTMatches12 = vector<int>((*ppDsrLKF)->mvKeysUn.size(),-1);//store index of Current Frame's keypoints
      for (size_t i=0;i<vBTMatches21.size();i++)
      {
        if(vBTMatches21[i]>=0)
        {
          vBTMatches12[vBTMatches21[i]] = i;
        }
      }
      if(nBoWmatches<10)
      {
        cout << "Not enough correspondences! Pose estimate may be not accurate." << endl;
        return 0;
      }


      if(mpInitializer != NULL)
      {
        cout <<"delete initializer"<<endl;
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
      }
      cout << "New initializer"<<endl;
      mpInitializer =  new Initializer((*ppDsrLKF)->mvKeysUn,mpCurrentFrame->mK,1.0,200);//mK is Calibration matrix
      cv::Mat Tcr = cv::Mat::eye(4,4,CV_32F);
      cv::Mat Rcr(3,3,CV_32F); // from current to reference
      cv::Mat tcr(3,1,CV_32F); // from current to reference
      vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
      cout<<"initialize"<<endl;
      if(mpInitializer->Initialize(mpCurrentFrame->mvKeysUn, vBTMatches12, Rcr, tcr, mvIniP3D, vbTriangulated))
      {
        for(size_t i=0, iend=vBTMatches12.size(); i<iend;i++)
        {
            if(vBTMatches12[i]>=0 && !vbTriangulated[i])
            {
              vBTMatches12[i]=-1;
              nBoWmatches--;
            }
        }
        BTlog<<", Triangulate: "<<nBoWmatches<<endl;
        Rcr.copyTo(Tcr.rowRange(0,3).colRange(0,3));
        tcr.copyTo(Tcr.rowRange(0,3).col(3));
        cv::Mat Tcbrb = mTbc*Tcr*mTcb;
        cv::Mat Tcgrg = mTgb*Tcbrb*mTbg;
        Tcr = mTwv.t()*Tcr*mTwv;
        cout << Tcr << endl;
        BTlog<< Tcr << endl;
        vector<float> q = Converter::toQuaternion(Tcgrg.rowRange(0,3).colRange(0,3));
        // geometry_msgs::PoseStamped Tcr_pose;
        // Tcr_pose.header.stamp = ros::Time::now();
        // Tcr_pose.header.frame_id = "Camera";
        // Tcr_pose.pose.position.x = tcr.at<float>(0);
        // Tcr_pose.pose.position.y = tcr.at<float>(1);
        // Tcr_pose.pose.position.z = tcr.at<float>(2);
        // Tcr_pose.pose.orientation.x = q[0];
        // Tcr_pose.pose.orientation.y = q[1];
        // Tcr_pose.pose.orientation.z = q[2];
        // Tcr_pose.pose.orientation.w = q[3];
        // Pub_Tcr.publish(Tcr_pose);
        geometry_msgs::TransformStamped Tcr_tf2;
        Tcr_tf2.header.stamp = ros::Time::now();
        Tcr_tf2.header.frame_id = "Current_Groundtruth";
        Tcr_tf2.child_frame_id = "Destination";
        Tcr_tf2.transform.translation.x = Tcgrg.at<float>(0,3);
        Tcr_tf2.transform.translation.y = Tcgrg.at<float>(1,3);
        Tcr_tf2.transform.translation.z = Tcgrg.at<float>(2,3);
        Tcr_tf2.transform.rotation.x = q[0];
        Tcr_tf2.transform.rotation.y = q[1];
        Tcr_tf2.transform.rotation.z = q[2];
        Tcr_tf2.transform.rotation.w = q[3];
        Pub_tf2.sendTransform(Tcr_tf2);
      }
      else
      {
        cout << Tcr << endl;
        BTlog<< Tcr << endl;
        // geometry_msgs::PoseStamped Tcr_pose;
        // Tcr_pose.header.stamp = ros::Time::now();
        // Tcr_pose.header.frame_id = "Camera";
        // Tcr_pose.pose.position.x = 0;
        // Tcr_pose.pose.position.y = 0;
        // Tcr_pose.pose.position.z = 0;
        // Tcr_pose.pose.orientation.x = 0;
        // Tcr_pose.pose.orientation.y = 0;
        // Tcr_pose.pose.orientation.z = 0;
        // Tcr_pose.pose.orientation.w = 1;
        // Pub_Tcr.publish(Tcr_pose);
        geometry_msgs::TransformStamped Tcr_tf2;
        Tcr_tf2.header.stamp = ros::Time::now();
        Tcr_tf2.header.frame_id = "Current_Groundtruth";
        Tcr_tf2.child_frame_id = "Destination";
        Tcr_tf2.transform.translation.x = 0;
        Tcr_tf2.transform.translation.y = 0;
        Tcr_tf2.transform.translation.z = 0;
        Tcr_tf2.transform.rotation.x = 0;
        Tcr_tf2.transform.rotation.y = 0;
        Tcr_tf2.transform.rotation.z = 0;
        Tcr_tf2.transform.rotation.w = 1;
        Pub_tf2.sendTransform(Tcr_tf2);

      }
      
      
    }
  return mnCurrentLKFId;

}



} //namespace ORB_SLAM

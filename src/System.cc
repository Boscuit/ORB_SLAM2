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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>


namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false),mbRecord(false),mbClear(true)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create LoadedKeyFrame Database
    mpLoadedKeyFrameDatabase = new LoadedKeyFrameDatabase(mpVocabulary);
    unsigned int nKFload = mpLoadedKeyFrameDatabase->LoadLKFFromTextFile("GroundTruth.csv","KeyFrameTrajectory.txt","KeyFrameKeyPointsUn.txt","KeyFrameKeyPoints.txt","KeyFrameDescriptor.txt","KeyFrameFeatureVector.txt","KeyFrameBowVector.txt");
    bool bDBload = mpLoadedKeyFrameDatabase->LoadDBFromTextFile("KeyFramevInvertedFile.txt");

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    mpBackTracker = new BackTracking(mpVocabulary, mpLoadedKeyFrameDatabase, mpTracker, mpFrameDrawer, nKFload, bDBload, strSettingsFile);
    if(mpBackTracker->isBackTrack() || mpBackTracker->isOnCommand())
    {
      mptBackTracking = new thread(&ORB_SLAM2::BackTracking::Run, mpBackTracker);
      mpTracker->SetBackTracker(mpBackTracker);
    }
    else
    {
      //Back Track mode: 0 or 1,2 but failed to load path;
      delete mpBackTracker;
      mpBackTracker = static_cast<BackTracking*>(NULL);
    }
      

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
        mbClear = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
        mbClear = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
        mbClear = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    // mTrackedKeyPointsDescriptor = mpTracker->mCurrentFrame.mDescriptors;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    if(mpBackTracker)
    {
      mpBackTracker->RequestFinish();
      while(!mpBackTracker->isFinished())
        usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose ilure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "TUM-trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "TUM-KEY-trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryEuRoc(const string &GroundTruthFile,const string &TrajectoryFile,const string &KeyPointsUnFile, const string &KeyPointsFile,
  const string &DescriptorsFile,const string &FeatureVectorFile,const string &BowVectorFile,
  const string &vInvertedFileFile,const string &MapPointsLocationFile,const string &MapPointsDescritorFile)
{
    cout << endl << "Saving ground truth to " << GroundTruthFile << " ..." << endl;
    cout << endl << "Saving keyframe trajectory to " << TrajectoryFile << " ..." << endl;
    cout << endl << "Saving keyframe undistorted key points to " << KeyPointsUnFile << " ..." << endl;
    cout << endl << "Saving keyframe key points to " << KeyPointsFile << " ..." << endl;
    cout << endl << "Saving keyframe descriptors to " << DescriptorsFile << " ..." << endl;
    cout << endl << "Saving keyframe FeatureVector to " << FeatureVectorFile << " ..." << endl;
    cout << endl << "Saving keyframe BowVector to " << BowVectorFile << " ..." << endl;
    cout << endl << "Saving keyframe vInvertedFile to " << vInvertedFileFile << " ..." << endl;
    cout << endl << "Saving keyframe MapPointsLocation to " << MapPointsLocationFile << " ..." << endl;
    cout << endl << "Saving keyframe MapPointsDescritor to " << MapPointsDescritorFile << " ..." << endl;

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    map<long unsigned int, cv::Mat> vKeyFramesIm = mpTracker->mvKeyFramesIm;
    map<double, vector<float> > vGroundTruth = mpTracker->mvGroundTruth;
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFramesNoCulling();
    // vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream gt,f,ku,k,d,fv,bv,vinv,mpl,mpd;
    gt.open(GroundTruthFile.c_str());
    f.open(TrajectoryFile.c_str());
    ku.open(KeyPointsUnFile.c_str());
    k.open(KeyPointsFile.c_str());
    d.open(DescriptorsFile.c_str());
    fv.open(FeatureVectorFile.c_str());
    bv.open(BowVectorFile.c_str());
    vinv.open(vInvertedFileFile.c_str());
    mpl.open(MapPointsLocationFile.c_str());
    mpd.open(MapPointsDescritorFile.c_str());
    gt   << fixed;
    f    << fixed;
    ku   << fixed;
    k    << fixed;
    d    << fixed;
    fv   << fixed;
    bv   << fixed;
    vinv << fixed;
    mpl  << fixed;
    mpd  << fixed;

    gt << "#timestamp, p_x [m], p_y [m], p_z [m], q_x [], q_y [], q_z [], q_w []" << endl;
    for(map<double,vector<float> >::iterator it=vGroundTruth.begin(), itend=vGroundTruth.end(); it!=itend; it++)
    {
      vector<float> tfv = it->second;
      gt << setprecision(6) << it->first <<","<<tfv[0] << ","<< tfv[1] << ","
      << tfv[2] << ","<< tfv[3] << ","<< tfv[4] << ","<< tfv[5] << ","<< tfv[6] << endl;
    }

    f << "#timestamp, p_x [m], p_y [m], p_z [m], q_x [], q_y [], q_z [], q_w []" << endl;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        // if(pKF->isBad())
        //     continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp
          << "," << t.at<float>(0) << "," << t.at<float>(1) << "," << t.at<float>(2)
          << "," << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << endl;

        ku << setprecision(1) <<  pKF->mnId << endl;
        vector<cv::KeyPoint> vKeysUn = pKF->mvKeysUn;
        for(size_t j=0; j<vKeysUn.size(); j++)
        {
          ku << setprecision(6) << vKeysUn[j].pt.x << " " << vKeysUn[j].pt.y << " " << vKeysUn[j].angle << " " << vKeysUn[j].octave << endl;
        }
        ku << "#";

        k << setprecision(1) << pKF->mnId << endl;
        vector<cv::KeyPoint> vKeys = pKF->mvKeys;
        for(size_t j=0; j<vKeys.size(); j++)
        {
          k << setprecision(6) << vKeys[j].pt.x << " " << vKeys[j].pt.y << " "<< vKeys[j].angle << " " << vKeys[j].octave << endl;
        }
        k << "#";

        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t j=0; j<vKeys.size(); j++)
        {
          MapPoint* pMP = vpMapPoints[j];
          if(!pMP || pMP->isBad())
            continue;
          cv::Mat MapPointLocation = pMP->GetWorldPos();
          mpl << j <<" "<<MapPointLocation.at<float>(0)<<" "<<MapPointLocation.at<float>(1)<<" "<<MapPointLocation.at<float>(2)<<" "<<endl;
          mpd << j <<" "<<pMP->GetDescriptor()<<endl;
        }
        mpl << "#";
        mpd << "#";

        d << setprecision(1)<< pKF->N << endl;
        d << pKF->mDescriptors;


        for(DBoW2::FeatureVector::const_iterator FVit=pKF->mFeatVec.begin(), FVend=pKF->mFeatVec.end(); FVit != FVend; FVit++)
        {
          fv << FVit->first << ",";
          vector<unsigned int> feature_vector = FVit->second;
          for (size_t j=0; j<feature_vector.size(); j++)
          {
            fv << feature_vector[j] << " ";
          }
          fv << endl;
        }
        fv << "#";

        for(DBoW2::BowVector::const_iterator BVit=pKF->mBowVec.begin(), BVend=pKF->mBowVec.end(); BVit != BVend; BVit++)
        {
          bv << setprecision(1)  << BVit->first << " " << setprecision(9) << BVit->second << " ";
        }
        bv << endl;

        try
        {
          stringstream name;
          name << "KeyImages/"<< pKF->mnId << ".png";
          cv::imwrite(name.str(),vKeyFramesIm[pKF->mnId],compression_params);
        }
        catch (runtime_error& ex) {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
        }
    }

    // for(map<long unsigned int, cv::Mat>::iterator vit=vKeyFramesIm.begin(),vend=vKeyFramesIm.end();vit!=vend;vit++)
    // {
    //   try
    //   {
    //     stringstream name;
    //     name << "KeyImages/"<< vit->first << ".png";
    //     cv::imwrite(name.str(),vit->second,compression_params);
    //   }
    //   catch (runtime_error& ex) {
    //       fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    //   }
    // }

    vector<vector<long unsigned int> > vInvertedFile = mpKeyFrameDatabase->GetvInvertedFileNoCulling();
    for(vector<vector<long unsigned int> >::iterator vit=vInvertedFile.begin(), vitend=vInvertedFile.end(); vit!=vitend; vit++)
    {
      vector<long unsigned int> vCommonKF = *vit;
      for(size_t i=0; i<vCommonKF.size(); i++)
      {
        vinv << setprecision(1) << vCommonKF[i] << " ";
      }
      vinv << endl;
    }

    gt.close();
    f.close();
    ku.close();
    k.close();
    d.close();
    fv.close();
    bv.close();
    vinv.close();
    mpl.close();
    mpd.close();
    cout << endl << "TUM-KEY-trajectory 2 saved!" << endl;
}


void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "KITTI-trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

// cv::Mat System::GetTrackedKeyPointsDescriptor()
// {
//     unique_lock<mutex> lock(mMutexState);
//     return mTrackedKeyPointsDescriptor;
// }

void System::StartRecord()
{
  unique_lock<mutex> lock(mMutexRecord);
  mbRecord = true;
  mpTracker->StartRecord();
}

void System::StopRecord()
{
  unique_lock<mutex> lock(mMutexRecord);
  mpTracker->StopRecord();
  mbRecord = false;
}

bool System::isRecording()
{
  unique_lock<mutex> lock(mMutexRecord);
  return mbRecord;
}

void System::BTactivate()
{
  if(!mpBackTracker)
  {
    cout << "BT is invalid." << endl;
    return;
  }
  if(!mpBackTracker->isOnCommand())
  {
    cout << "BT is not on command." << endl;
    return;
  }
  mpBackTracker->Activate(mpMap,mpKeyFrameDatabase);
}

void System::BTrequestStop()
{
  if(!mpBackTracker)
  {
    cout << "BT is invalid." << endl;
    return;
  }
  if(!mpBackTracker->isOnCommand())
  {
    cout << "BT is not on command." << endl;
    return;
  }
  mpBackTracker->RequestStop();
}

void System::AddGroundTruth(const double &timestamp, const vector<float> &groundtruth)
{
  mpTracker->mvGroundTruth.insert(pair<double,vector<float> >(timestamp,groundtruth));
}

vector<vector<float> > System::GetvSimilarityMatches()
{
  unique_lock<mutex> lock(mpTracker->mMutexSimilarityMatches);
  return mpTracker->mvSimilarityMatches;
}

vector<float> System::Twc2sevenD(cv::Mat Twc)
{
  vector<float> vPubPose;
  cv::Mat twc(3,1,CV_32F);
  cv::Mat Rwc(3,3,CV_32F);
  Rwc = Twc.rowRange(0,3).colRange(0,3);
  twc = Twc.rowRange(0,3).col(3);
  vector<float> q = Converter::toQuaternion(Rwc);
  vPubPose.push_back (twc.at<float>(0));
  vPubPose.push_back (twc.at<float>(1));
  vPubPose.push_back (twc.at<float>(2));
  vPubPose.push_back (q[0]);
  vPubPose.push_back (q[1]);
  vPubPose.push_back (q[2]);
  vPubPose.push_back (q[3]);

  return vPubPose;
}

cv::Mat System::sevenD2Twc(vector<float> sevenD)
{
  cv::Mat Twc(4,4,CV_32F);
  vector<float> q(sevenD.begin()+3,sevenD.end());
  cv::Mat Rwc = Converter::toCvMat(q);
  Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
  Twc.at<float>(0,3) = sevenD[0];
  Twc.at<float>(1,3) = sevenD[1];
  Twc.at<float>(2,3) = sevenD[2];
  Twc.at<float>(3,0) = 0;
  Twc.at<float>(3,1) = 0;
  Twc.at<float>(3,2) = 0;
  Twc.at<float>(3,3) = 1;

  return Twc;
}


cv::Mat System::InverseT(cv::Mat Tcw)
{
  cv::Mat twc(3,1,CV_32F);
  cv::Mat Rwc(3,3,CV_32F);
  cv::Mat Twc(4,4,CV_32F);
  Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
  twc = -Rwc*Tcw.rowRange(0,3).col(3);
  Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
  twc.copyTo(Twc.rowRange(0,3).col(3));
  Twc.at<float>(3,0) = 0;
  Twc.at<float>(3,1) = 0;
  Twc.at<float>(3,2) = 0;
  Twc.at<float>(3,3) = 1;
  return Twc;
}

vector<cv::Mat> System::GetKeyCameraPoseVector()
{
  vector<cv::Mat> vKeyPose;
  vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
  for(size_t i=0; i<vpKFs.size(); i++)
  {
      KeyFrame* pKF = vpKFs[i];
      cv::Mat Twc = pKF->GetPoseInverse();
      vKeyPose.push_back(Twc);
  }
  return vKeyPose;
}

bool System::isClear()
{
  if(!mbClear)
  {
    mbClear = true;
    return false;
  }
  else return true;
}

void System::CompareImgs(cv::Mat Im1, cv::Mat Im2)
{
  mpTracker->CompareImgs(Im1, Im2);
}


} //namespace ORB_SLAM

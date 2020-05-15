#include "LoadedKeyFrameDatabase.h"

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

LoadedKeyFrameDatabase::LoadedKeyFrameDatabase (ORBVocabulary* pVoc):mpLoadedVoc(pVoc)
{
}


unsigned int LoadedKeyFrameDatabase::LoadLKFFromTextFile (const string &GroundTruthFile,const string &TrajectoryFile,const string &KeyPointsUnFile,const string &KeyPointsFile,const string &DescriptorsFile,const string &FeatureVectorFile,const string &BowVectorFile)
{
    ifstream ku(KeyPointsUnFile);
    ifstream k(KeyPointsFile);
    cout<<"loadingLKF"<<endl;

    string KFline;
    string KPsline;
    // int countKF=0;
    // vector<float> vKPx;
    // vector<float> vKPy;
    // float KPxy;
    bool bEvenline=true;
    float KPx;
    float KPy;
    float KPangle;
    int KPoctave;
    long unsigned int LKFid;//id of loaded key frame in last experiment
    vector<long unsigned int> vLKFid;//vector of id of loaded key frame in last experiment
    vector<cv::KeyPoint> vKeysUn;
    vector<cv::KeyPoint> vKeys;
    vector<vector<cv::KeyPoint>> vvKeysUn;
    vector<vector<cv::KeyPoint>> vvKeys;

    //load frame id and key points Undistored
    while (getline(ku,KFline,'#'))
    {
      stringstream sKFline(KFline);
      getline(sKFline,KPsline);
      LKFid = atoi(KPsline.c_str());
      vLKFid.push_back(LKFid);
      while (getline(sKFline,KPsline))
      {
        cv::KeyPoint kp;
        sscanf(KPsline.c_str(), "%f%f%f%d", &kp.pt.x, &kp.pt.y, &kp.angle, &kp.octave);
        // cv::KeyPoint kp;
        // kp.pt.x = KPx;
        // kp.pt.y = KPy;
        // kp.angle = KPangle;
        // kp.octave = KPoctave;
        vKeysUn.push_back(kp);
      }
      vvKeysUn.push_back(vKeysUn);
      vKeysUn.clear();
    }
    // if(vvKeysUn.size()!=0)
    // {
    //   vKeysUn = vvKeysUn[0];
    //   for (size_t i=0;i<10;i++)
    //   {
    //     cout<<vKeysUn[i].pt.x << " " << vKeysUn[i].pt.y << " " << vKeysUn[i].angle << " " << vKeysUn[i].octave << endl;
    //   }
    //   for(size_t i=0;i<vvKeysUn.size();i++)
    //   {
    //     cout << "size of vKeysUn:" << vvKeysUn[i].size()<<endl;
    //   }
    // }

    cout<<"size of vvKeysUn:"<<vvKeysUn.size()<<endl;
    if(vvKeysUn.size()==0)
      return 0;

    //load frame id and key points
    bEvenline=true;
    while (getline(k,KFline,'#'))
    {
      stringstream sKFline(KFline);
      getline(sKFline,KPsline);
      while (getline(sKFline,KPsline))
      {
        cv::KeyPoint kp;
        sscanf(KPsline.c_str(), "%f%f%f%d", &kp.pt.x, &kp.pt.y, &kp.angle, &kp.octave);
        // cv::KeyPoint kp;
        // kp.pt.x = KPx;
        // kp.pt.y = KPy;
        // kp.angle = KPangle;
        // kp.octave = KPoctave;
        vKeys.push_back(kp);
      }
      vvKeys.push_back(vKeys);
      vKeys.clear();
    }
    // if(vvKeys.size()!=0)
    // {
    //   vKeys = vvKeys[0];
    //   for (size_t i=0;i<10;i++)
    //   {
    //     cout<<vKeys[i].pt.x << " " << vKeys[i].pt.y << " " << vKeys[i].angle << " " << vKeys[i].octave << endl;
    //   }
    //   for(size_t i=0;i<vvKeys.size();i++)
    //   {
    //     cout << "size of vKeys:" << vvKeys[i].size()<<endl;
    //   }
    // }

    // //load frame id and key points
    // while (getline(k,KPsline))
    // {
    //   int countKP=0;
    //   stringstream sKPsline(KPsline);
    //   if(countKF%2 != 0)
    //   {
    //     while (sKPsline >> KPxy)
    //     {
    //       if (countKP%2 == 0)
    //         vKPx.push_back(KPxy);
    //       else
    //       {
    //         vKPy.push_back(KPxy);
    //       }
    //       countKP++;
    //     }
    //     //push KP
    //     for (size_t i=0; i<vKPx.size(); i++)
    //     {
    //       cv::KeyPoint KP;
    //       KP.pt.x=vKPx[i];
    //       KP.pt.y=vKPy[i];
    //       vKeys.push_back(KP);
    //     }
    //     vvKeys.push_back(vKeys);
    //     vKPx.clear();
    //     vKPy.clear();
    //     vKeys.clear();
    //   }
    //   countKF++;
    // }
    // countKF=0;

    cout<<"size of vvKeys:"<<vvKeys.size()<<endl;
    if(vvKeys.size()==0)
      return 0;

    //load GroundTruth trajectory
    map<double,vector<float> > vGroundTruth = LoadTrajectoryFromTextFile (GroundTruthFile);
    if(vGroundTruth.size()==0)
      return 0;

    //load estimated trajectory
    map<double,vector<float> > vEstimatedPose = LoadTrajectoryFromTextFile (TrajectoryFile);
    if(vEstimatedPose.size()==0)
      return 0;
    else if(vEstimatedPose.begin()->first > vGroundTruth.rbegin()->first)
    {
      cerr << "Trajectory(begin at "<<vEstimatedPose.begin()->first<<") does not match GroundTruth(end at " <<vGroundTruth.rbegin()->first<< ")" <<endl;
      return 0;
    }
    else if(vGroundTruth.begin()->first > vEstimatedPose.rbegin()->first)
    {
      cerr << "GroundTruth(begin at "<<vGroundTruth.begin()->first<<") does not match Trajectory(end at " <<vEstimatedPose.rbegin()->first<< ")" <<endl;
      return 0;
    }


    //load descriptors
    vector<cv::Mat> vDescriptors = LoadLKFDescriptorFromTextFile(DescriptorsFile);
    if(vDescriptors.size()==0)
      return 0;

    //load FeatureVector
    vector<DBoW2::FeatureVector> vFeatVec = LoadFeatureVectorFromTextFile(FeatureVectorFile);
    if(vFeatVec.size()==0)
      return 0;

    //load BowVector
    vector<DBoW2::BowVector> vBowVec = LoadBowVectorFromTextFile (BowVectorFile);
    if(vBowVec.size()==0)
      return 0;

    if(vEstimatedPose.size()!=vvKeysUn.size() || vDescriptors.size()!=vvKeysUn.size() ||
        vFeatVec.size()!=vvKeysUn.size() || vBowVec.size()!=vvKeysUn.size() || vvKeys.size()!=vvKeysUn.size())
        return 0;

    //construct LoadedKeyFrame
    int countKF = 0;
    for(map<double,vector<float> >::iterator it=vEstimatedPose.begin(), itend=vEstimatedPose.end(); it!=itend; it++)
    {
      double TimeStamp = it->first;
      map<double,vector<float>>::iterator tempit = vGroundTruth.lower_bound(TimeStamp);
      if(tempit==vGroundTruth.end())//incase the timestamp of LKF is larger than what GoundTruth has
        tempit--;
      // cout << "Constructing LKF: "<< fixed << tempit->first;
      vector<float> GoundTruth = tempit->second;
      // cout << " Found GroundTruth. " << countKF <<endl;
      // int nLKF = it-vEstimatedPose.begin();//??? error: no match for ‘operator-’
      int nLKF = countKF;
      LKFid = vLKFid[nLKF];
      vKeysUn = vvKeysUn[nLKF];
      vKeys = vvKeys[nLKF];
      int N = vKeys.size();
      cv::Mat Descriptors = vDescriptors[nLKF];
      DBoW2::BowVector BowVec = vBowVec[nLKF];
      DBoW2::FeatureVector FeatVec = vFeatVec[nLKF];
      LoadedKeyFrame* pLKF = new LoadedKeyFrame(TimeStamp,LKFid,N,vKeysUn,vKeys,Descriptors,BowVec,FeatVec,GoundTruth);
      mvpLoaedKeyFrame.insert(pair<long unsigned int,LoadedKeyFrame*>(LKFid,pLKF));
      countKF++;
    }

    cout<<"size of mvpLoaedKeyFrame:"<<mvpLoaedKeyFrame.size()<<endl;
    // for(std::map<long unsigned int,LoadedKeyFrame*>::const_iterator it=mvpLoaedKeyFrame.begin(), itend=mvpLoaedKeyFrame.end(); it != itend; it++)
    // {
    //   LoadedKeyFrame* temp = it->second;
    //   cout<<temp->mnId<<endl;
    // }

    //load Images
    for(size_t i=0;i<vLKFid.size();i++)
    {
      cv::Mat tempIm;
      stringstream name;
      name << "KeyImages/"<< vLKFid[i] << ".png";
      tempIm = cv::imread(name.str(),CV_LOAD_IMAGE_UNCHANGED);
      if(tempIm.empty())
      {
        cerr << endl << "Failed to load image at: " <<  name.str() << endl;
        return 0;
      }
      if(tempIm.channels()==3)
      {
        cvtColor(tempIm,tempIm,CV_RGB2GRAY);
      }
      mvLoadedImages.insert(pair<long unsigned int, cv::Mat>(vLKFid[i],tempIm));
    }
    cout << "size of mvLoadedImages: "<<mvLoadedImages.size()<<endl;
    if(mvLoadedImages.size()==0)
      return 0;

    return mvpLoaedKeyFrame.size();
}


bool LoadedKeyFrameDatabase::LoadDBFromTextFile (const string &vInvertedFileFile)
{
  ifstream inv(vInvertedFileFile);
  string line;
  int nline=1;
  long unsigned int LKFid;//id of loaded key frame in last experiment
  list<LoadedKeyFrame*> LoadedInvertedFile;
  if (!mvLoadedInvertedFile.empty())
    mvLoadedInvertedFile.clear();
  while(getline(inv,line))
  {
    stringstream sline(line);
    while(sline >> LKFid)
    {
        // cout << LKFid << " ";
        LoadedKeyFrame* pLKF = mvpLoaedKeyFrame.find(LKFid)->second;
        LoadedInvertedFile.push_back(pLKF);
    }
    mvLoadedInvertedFile.push_back(LoadedInvertedFile);
    LoadedInvertedFile.clear();
    nline++;
  }
  if(mvLoadedInvertedFile.size()!=mpLoadedVoc->size())
  {
    cout << "Loading last database failed. Initializing..." << endl;
    mvLoadedInvertedFile.resize(mpLoadedVoc->size());
    return false;
  }
  cout<<"size of mvLoadedInvertedFile:"<<mvLoadedInvertedFile.size()<<endl;
  // for(size_t i=0;i<mvLoadedInvertedFile.size();i++)
  // {
  //   LoadedInvertedFile = mvLoadedInvertedFile[i];
  //   if(!LoadedInvertedFile.empty())
  //   {
  //     cout<<i<<": ";
  //     for(std::list<LoadedKeyFrame* >::const_iterator it=LoadedInvertedFile.begin(), itend=LoadedInvertedFile.end(); it != itend; it++)
  //     {
  //       LoadedKeyFrame* pLKF = *it;
  //       cout<<pLKF->mnId<<" ";
  //     }
  //     cout <<endl;
  //   }
  // }
  return true;
}


vector<LoadedKeyFrame*> LoadedKeyFrameDatabase::DetectBackTrackCandidates(Frame *F, unsigned int nMaxCan)
{
    // cout <<"F BowVec: "<<F->mBowVec.size()<<endl;

    list<LoadedKeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            list<LoadedKeyFrame*> &lKFs = mvLoadedInvertedFile[vit->first];
            // cout << "lKFs.size: " << lKFs.size() <<endl;
            for(list<LoadedKeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                LoadedKeyFrame* pLKFi=*lit;
                if(pLKFi==NULL)
                {
                  // cout << "NULL pLKFi at word: " << vit->first <<endl;
                  continue;
                }
                if(pLKFi->mnBackTrackQuery!=F->mnId)
                {
                    pLKFi->mnBackTrackWords=0;
                    pLKFi->mnBackTrackQuery=F->mnId;
                    lKFsSharingWords.push_back(pLKFi);
                }
                pLKFi->mnBackTrackWords++;
            }
        }
    }
    // cout<<"size of lKFsSharingWords: "<<lKFsSharingWords.size()<<endl;

    if(lKFsSharingWords.empty())
        return vector<LoadedKeyFrame*>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<LoadedKeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnBackTrackWords>maxCommonWords)
            maxCommonWords=(*lit)->mnBackTrackWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    vector<LoadedKeyFrame*> vScoredMatch;//modified code
    mspScoredLKF.clear();

    int nscores=0;

    // Compute similarity score.
    for(list<LoadedKeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        LoadedKeyFrame* pLKFi = *lit;

        // if(pLKFi->mnBackTrackWords>minCommonWords)
        // {
            nscores++;
            float si = mpLoadedVoc->score(F->mBowVec,pLKFi->mBowVec);
            pLKFi->mBackTrackScore=si;
            mspScoredLKF.insert(pLKFi);
            if(pLKFi->mnBackTrackWords>minCommonWords)
            {
            vScoredMatch.push_back(pLKFi);
            }
    }
    // cout << "vScoredMatch.size: " << vScoredMatch.size() <<endl;

    if(vScoredMatch.empty())
        return vector<LoadedKeyFrame*>();

    sort(vScoredMatch.begin(),vScoredMatch.end(),LoadedKeyFrame::largerScore);//larger score come first


    if(vScoredMatch.size()>nMaxCan)
    {
      vector<LoadedKeyFrame*>::const_iterator vit=vScoredMatch.begin();
      vector<LoadedKeyFrame*>::const_iterator vitend=vScoredMatch.begin()+nMaxCan;
      vector<LoadedKeyFrame*> vpBackTrackCandidates(vit,vitend);
      return vpBackTrackCandidates;
    }
    else
    {
      return vScoredMatch;
    }

    // list<pair<float,LoadedKeyFrame*> > lAccScoreAndMatch;
    // float bestAccScore = 0;
    //
    // // Lets now accumulate score by covisibility
    // for(list<pair<float,LoadedKeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    // {
    //     LoadedKeyFrame* pLKFi = it->second;
    //     vector<LoadedKeyFrame*> vpNeighs = pLKFi->GetBestCovisibilityKeyFrames(10);
    //
    //     float bestScore = it->first;
    //     float accScore = bestScore;
    //     LoadedKeyFrame* pBestLKF = pLKFi;
    //     for(vector<LoadedKeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
    //     {
    //         LoadedKeyFrame* pKF2 = *vit;
    //         if(pKF2->mnBackTrackQuery!=F->mnId)
    //             continue;
    //
    //         accScore+=pKF2->mBackTrackScore;
    //         if(pKF2->mBackTrackScore>bestScore)
    //         {
    //             pBestLKF=pKF2;
    //             bestScore = pKF2->mBackTrackScore;
    //         }
    //
    //     }
    //     lAccScoreAndMatch.push_back(make_pair(accScore,pBestLKF));
    //     if(accScore>bestAccScore)
    //         bestAccScore=accScore;
    // }
    //
    // // Return all those keyframes with a score higher than 0.75*bestScore
    // float minScoreToRetain = 0.75f*bestAccScore;
    // set<LoadedKeyFrame*> spAlreadyAddedLKF;
    // vector<LoadedKeyFrame*> vpBackTrackCandidates;
    // vpBackTrackCandidates.reserve(lAccScoreAndMatch.size());
    // for(list<pair<float,LoadedKeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    // {
    //     const float &si = it->first;
    //     if(si>minScoreToRetain)
    //     {
    //         LoadedKeyFrame* pLKFi = it->second;
    //         if(!spAlreadyAddedLKF.count(pLKFi))
    //         {
    //             vpBackTrackCandidates.push_back(pLKFi);
    //             spAlreadyAddedLKF.insert(pLKFi);
    //         }
    //     }
    // }

}


map<long unsigned int, float> LoadedKeyFrameDatabase::GetSimilarity()
{
  map<long unsigned int, float> vIdAndScore;
  for(map<long unsigned int,LoadedKeyFrame*>::iterator vit=mvpLoaedKeyFrame.begin(), vend=mvpLoaedKeyFrame.end(); vit!=vend; vit++)
  {
    LoadedKeyFrame* pLKFi = vit->second;
    if(mspScoredLKF.count(pLKFi))
      vIdAndScore.insert(pair<long unsigned int,float>(pLKFi->mnId,pLKFi->mBackTrackScore));
    else
      vIdAndScore.insert(pair<long unsigned int,float>(pLKFi->mnId,0));
  }
  return vIdAndScore;
}

LoadedKeyFrame* LoadedKeyFrameDatabase::GetNextLKF(const long unsigned int nCurrentId, bool bForward)
{
  long unsigned int nNextId;
  if(bForward)
    nNextId = nCurrentId+1;
  else
    nNextId = nCurrentId-1;
  map<long unsigned int,LoadedKeyFrame*>::iterator it = mvpLoaedKeyFrame.find(nNextId);
  if(it != mvpLoaedKeyFrame.end())
    return it->second;
  else
    return mvpLoaedKeyFrame.find(nCurrentId)->second;
}

bool LoadedKeyFrameDatabase::IsLast(const long unsigned int nCurrentId, bool bForward)
{
  if(bForward)
    return (nCurrentId == mvpLoaedKeyFrame.rbegin()->first);
  else
   return (nCurrentId == mvpLoaedKeyFrame.begin()->first);
}


map<double,vector<float> > LoadedKeyFrameDatabase::LoadTrajectoryFromTextFile (const string &TrajectoryFile)
{
  double t;
  float px, py, pz, qx, qy, qz, qw;
  map<double, vector<float> > vPose;
  FILE * pTrajectoryFile = fopen(TrajectoryFile.c_str(),"r");
  if(pTrajectoryFile==NULL)
  {
    cerr << "can't load ground truth; wrong path " << TrajectoryFile << endl;
    return vPose;
  }
  char tmp[10000];
  if (fgets(tmp, 10000, pTrajectoryFile) == NULL)
  {
    cerr << "can't load " << TrajectoryFile << "; no data available" << endl;
    return vPose;
  }
  while (!feof(pTrajectoryFile))
  {
    if(fscanf(pTrajectoryFile, "%lf,%f,%f,%f,%f,%f,%f,%f", &t,
            &px, &py, &pz, &qx, &qy, &qz, &qw) != EOF)
    {
      vector<float> Pose;
      Pose.push_back(px);
      Pose.push_back(py);
      Pose.push_back(pz);
      Pose.push_back(qx);
      Pose.push_back(qy);
      Pose.push_back(qz);
      Pose.push_back(qw);
      vPose.insert(pair<double,vector<float> >(t,Pose));
      // printf("Read: %lf,%f,%f,%f,%f,%f,%f,%f\n",t,px,py,pz,qx,qy,qz,qw);
    }
  }
  fclose(pTrajectoryFile);
  cout << "size of Trajectory: "<< vPose.size()<<endl;
  return vPose;
}

vector<cv::Mat> LoadedKeyFrameDatabase::LoadLKFDescriptorFromTextFile (const string &DescriptorsFile)
{
  int N;//number of key points in this frame
  int e;
  int row=0;
  int col=0;
  string Descriptorsblock;
  string strRow;
  string strN;
  string element;
  vector<cv::Mat> vDescriptors;

  ifstream d(DescriptorsFile);

  while (getline(d,Descriptorsblock,']'))
  {
    stringstream sDescriptorsblock(Descriptorsblock);
    getline(sDescriptorsblock,strN);
    N = atoi(strN.c_str());
    cv::Mat Descriptors(N,32,CV_8UC1);
    while (getline(sDescriptorsblock,strRow))
    {
      if (row == 0)  strRow.erase(0,1);
      stringstream sstrRow(strRow);
      while (getline(sstrRow,element,','))
      {
        e = atoi(element.c_str());
        Descriptors.at<unsigned char>(row,col) = (unsigned char)e;
        col++;
      }
      row++;
      col=0;
    }
    vDescriptors.push_back(Descriptors);
    row=0;
    N=0;
  }
  cout<<"size of vDescriptors:"<<vDescriptors.size()<<endl;

  return vDescriptors;
}

vector<DBoW2::FeatureVector> LoadedKeyFrameDatabase::LoadFeatureVectorFromTextFile (const string &FeatureVectorFile)
{
  unsigned int feature;
  DBoW2::NodeId nid;//id of node in FeatureVector
  string snid;
  string FeatureVectorblock;
  string FeatureVectorline;
  vector<DBoW2::FeatureVector> vFeatVec;

  ifstream fv(FeatureVectorFile);

  while (getline(fv,FeatureVectorblock,'#'))
  {
    stringstream sFeatureVectorblock(FeatureVectorblock);
    DBoW2::FeatureVector tempFeatVec;
    while (getline(sFeatureVectorblock,FeatureVectorline))
    {
      stringstream sFeatureVectorline(FeatureVectorline);
      getline(sFeatureVectorline,snid,',');
      nid = atoi(snid.c_str());
      while (sFeatureVectorline >> feature)
      {
        tempFeatVec.addFeature(nid,feature);
      }
    }
    vFeatVec.push_back(tempFeatVec);
  }
  cout<<"size of vFeatVec:"<<vFeatVec.size()<<endl;
  // cout<<"get first fv"<<endl;
  // DBoW2::FeatureVector firstfeatvec = vFeatVec[0];
  // for(DBoW2::FeatureVector::const_iterator FVit=firstfeatvec.begin(), FVend=firstfeatvec.end(); FVit != FVend; FVit++)
  // {
  //   cout << FVit->first << ",";
  //   vector<unsigned int> feature_vector = FVit->second;
  //   for (size_t j=0; j<feature_vector.size(); j++)
  //   {
  //     cout << feature_vector[j] << " ";
  //   }
  //   cout << endl;
  // }
  // cout<<endl;
  return vFeatVec;
}

vector<DBoW2::BowVector> LoadedKeyFrameDatabase::LoadBowVectorFromTextFile (const string &BowVectorFile)
{
  bool even=true;
  double word;
  string BowVectorblock;
  DBoW2::WordId wid;
  DBoW2::WordValue wv;
  vector<DBoW2::BowVector> vBowVec;

  ifstream bv(BowVectorFile);

  while (getline(bv,BowVectorblock))
  {
    stringstream sBowVectorblock(BowVectorblock);
    DBoW2::BowVector tempBowVec;
    while (sBowVectorblock >> word)
    {
      if(even)
      {
        wid = (int)word;
        even = false;
      }
      else
      {
        wv = word;
        tempBowVec.addIfNotExist(wid,wv);
        even = true;
      }
    }
    vBowVec.push_back(tempBowVec);
  }
  cout<<"size of vBowVec:"<<vBowVec.size()<<endl;
  // for(size_t i=0; i<vBowVec.size();i++)
  // {
  //   cout<< "size of tempBowVec:" << vBowVec[i].size()<<endl;
  // }
  return vBowVec;
}

} //namespace ORB_SLAM

#include "LoadedKeyFrameDatabase.h"

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


void LoadedKeyFrameDatabase::LoadLKFFromTextFile (const string &TrajectoryFile,const string &KeyPointsFile,const string &DescriptorsFile,const string &FeatureVectorFile,const string &BowVectorFile)
{
    ifstream f(TrajectoryFile);
    ifstream k(KeyPointsFile);
    cout<<"loadingLKF"<<endl;

    string KFline;
    string KPsline;
    int countKF=0;
    vector<float> vKPx;
    vector<float> vKPy;
    float KPxy;
    long unsigned int LKFid;//id of loaded key frame in last experiment
    vector<long unsigned int> vLKFid;//vector of id of loaded key frame in last experiment
    vector<cv::KeyPoint> vKeys;
    vector<vector<cv::KeyPoint>> vvKeys;

    //load frame id and key points
    while (getline(k,KPsline))
    {
      int countKP=0;
      stringstream sKPsline(KPsline);
      if (countKF%2 == 0)//even line indicate id
      {
        sKPsline >> LKFid;
        vLKFid.push_back(LKFid);
      }
      else
      {
        while (sKPsline >> KPxy)
        {
          if (countKP%2 == 0)
            vKPx.push_back(KPxy);
          else
          {
            vKPy.push_back(KPxy);
          }
          countKP++;
        }
        //push KP
        for (size_t i=0; i<vKPx.size(); i++)
        {
          cv::KeyPoint KP;
          KP.pt.x=vKPx[i];
          KP.pt.y=vKPy[i];
          vKeys.push_back(KP);
        }
        vvKeys.push_back(vKeys);
        vKeys.clear();
      }
      countKF++;
    }
    countKF=0;

    cout<<"size of vvKeys:"<<vvKeys.size()<<endl;

    //load descriptors
    vector<cv::Mat> vDescriptors = LoadLKFDescriptorFromTextFile(DescriptorsFile);

    //load FeatureVector
    vector<DBoW2::FeatureVector> vFeatVec = LoadFeatureVectorFromTextFile(FeatureVectorFile);

    //load BowVector
    vector<DBoW2::BowVector> vBowVec = LoadBowVectorFromTextFile (BowVectorFile);

    //construct LoadedKeyFrame
    for(size_t nLKF=0;nLKF<vvKeys.size();nLKF++)
    {
      LKFid = vLKFid[nLKF];
      vKeys = vvKeys[nLKF];
      int N = vKeys.size();
      cv::Mat Descriptors = vDescriptors[nLKF];
      DBoW2::BowVector BowVec = vBowVec[nLKF];
      DBoW2::FeatureVector FeatVec = vFeatVec[nLKF];
      LoadedKeyFrame* pLKF = new LoadedKeyFrame(LKFid,N,vKeys,Descriptors,BowVec,FeatVec);
      vpLoaedKeyFrame.insert(pair<long unsigned int,LoadedKeyFrame*>(LKFid,pLKF));
    }
    cout<<"size of vpLoaedKeyFrame:"<<vpLoaedKeyFrame.size()<<endl;
    // for(std::map<long unsigned int,LoadedKeyFrame*>::const_iterator it=vpLoaedKeyFrame.begin(), itend=vpLoaedKeyFrame.end(); it != itend; it++)
    // {
    //   cout<<it->first<<endl;
    // }
}


void LoadedKeyFrameDatabase::LoadDBFromTextFile (const string &vInvertedFileFile)
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
        LoadedKeyFrame* pLKF = vpLoaedKeyFrame.find(LKFid)->second;
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
}


vector<LoadedKeyFrame*> LoadedKeyFrameDatabase::DetectBackTrackCandidates(Frame *F, int nMaxCan)
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
    // cout << "lKFsSharingWords.size: " << lKFsSharingWords.size() <<endl;
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

    int nscores=0;

    // Compute similarity score.
    for(list<LoadedKeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        LoadedKeyFrame* pLKFi = *lit;

        if(pLKFi->mnBackTrackWords>minCommonWords)
        {
            nscores++;
            float si = mpLoadedVoc->score(F->mBowVec,pLKFi->mBowVec);
            pLKFi->mBackTrackScore=si;
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
    // vN.push_back(N);
    cv::Mat Descriptors(N,32,CV_8UC1);
    while (getline(sDescriptorsblock,strRow))
    {
      if (row == 0)  strRow.erase(0,1);
      stringstream sstrRow(strRow);
      while (getline(sstrRow,element,','))
      {
        // cout << "step3";
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

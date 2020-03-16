#ifndef LOADEDKEYFRAMEDATABASE_H
#define LOADEDKEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "LoadedKeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

class LoadedKeyFrame;
class Frame;

class LoadedKeyFrameDatabase
{
public:

  LoadedKeyFrameDatabase(ORBVocabulary* pVoc);

  bool LoadDBFromTextFile (const string &vInvertedFileFile);

  unsigned int LoadLKFFromTextFile (const string &GroundTruthFile, const string &TrajectoryFile,const string &KeyPointsFile,const string &DescriptorsFile,const string &FeatureVectorFile,const string &BowVectorFile);


  //BackTrack candidate given by BowScore
  std::vector<LoadedKeyFrame*> DetectBackTrackCandidates(Frame *F, unsigned int nMaxCan);

  std::map<long unsigned int, float> GetSimilarity();

private:
  std::map<double,vector<float> > LoadTrajectoryFromTextFile (const string &GroundTruthFile,const float offset);

  std::vector<cv::Mat> LoadLKFDescriptorFromTextFile (const string &DescriptorsFile);

  std::vector<DBoW2::FeatureVector> LoadFeatureVectorFromTextFile (const string &FeatureVectorFile);

  std::vector<DBoW2::BowVector> LoadBowVectorFromTextFile (const string &BowVectorFile);

  // std::vector<vector<cv::Mat> > LoadMapPointLocationFromTextFile ()



protected:

  // Associated vocabulary
  ORBVocabulary* mpLoadedVoc;

  std::map<long unsigned int,LoadedKeyFrame*> mvpLoaedKeyFrame;//<nId,pLoadedKeyFrame>

  std::set<LoadedKeyFrame*> mspScoredLKF;

  // Inverted file
  std::vector<list<LoadedKeyFrame*> > mvLoadedInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif

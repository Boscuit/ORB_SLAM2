# include "LoadedKeyFrame.h"

namespace ORB_SLAM2
{
LoadedKeyFrame::LoadedKeyFrame(double TimeStamp, long unsigned int nId, int N, std::vector<cv::KeyPoint> vKeysUn, std::vector<cv::KeyPoint> vKeys, cv::Mat Descriptors, DBoW2::BowVector BowVec, DBoW2::FeatureVector FeatVec, std::vector<float> GroundTruth):
    mTimeStamp(TimeStamp),mnId(nId),mN(N),mvKeysUn(vKeysUn),mvKeys(vKeys),mDescriptors(Descriptors),mBowVec(BowVec),mFeatVec(FeatVec),mGroundTruth(GroundTruth)
{
}

}

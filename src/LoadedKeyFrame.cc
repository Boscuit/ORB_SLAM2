# include "LoadedKeyFrame.h"

namespace ORB_SLAM2
{
LoadedKeyFrame::LoadedKeyFrame(long unsigned int nId, int N, std::vector<cv::KeyPoint> vKeys, cv::Mat Descriptors, DBoW2::BowVector BowVec, DBoW2::FeatureVector FeatVec):
    mnId(nId),mN(N),mvKeys(vKeys),mDescriptors(Descriptors),mBowVec(BowVec),mFeatVec(FeatVec)
{
}

}

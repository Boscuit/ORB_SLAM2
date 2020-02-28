#ifndef LOADEDKEYFRAME_H
#define LOADEDKEYFRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

namespace ORB_SLAM2
{

class Map;
class MapPoint;

class LoadedKeyFrame
{
public:
    LoadedKeyFrame(long unsigned int nId, int N, std::vector<cv::KeyPoint> vKeys, cv::Mat Descriptors, DBoW2::BowVector BowVec, DBoW2::FeatureVector FeatVec);

    static bool largerScore(LoadedKeyFrame* pLKF1, LoadedKeyFrame* pLKF2){
        return pLKF1->mBackTrackScore>pLKF2->mBackTrackScore;
    }

public:
    long unsigned int mnId;
    int mN;
    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    long unsigned int mnBackTrackQuery;
    int mnBackTrackWords;
    float mBackTrackScore;
};


}//namespace ORB_SLAM2

#endif

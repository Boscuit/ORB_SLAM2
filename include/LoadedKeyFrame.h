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
    LoadedKeyFrame(double TimeStamp, long unsigned int nId, int N, std::vector<cv::KeyPoint> vKeysUn, std::vector<cv::KeyPoint> vKeys,
       cv::Mat Descriptors, DBoW2::BowVector BowVec, DBoW2::FeatureVector FeatVec, std::vector<float> GroundTruth);

    static bool largerScore(LoadedKeyFrame* pLKF1, LoadedKeyFrame* pLKF2){
        return pLKF1->mBackTrackScore>pLKF2->mBackTrackScore;
    }

    static bool lId(LoadedKeyFrame* pLKF1, LoadedKeyFrame* pLKF2){
    return pLKF1->mnId<pLKF2->mnId;
    }

public:
    double mTimeStamp;
    long unsigned int mnId;
    int mN;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    long unsigned int mnBackTrackQuery;
    int mnBackTrackWords;
    float mBackTrackScore;

    std::vector<float> mGroundTruth;
};


}//namespace ORB_SLAM2

#endif

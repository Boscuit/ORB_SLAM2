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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap),mnfId(0)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mRefIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mSimilarityGraph = cv::Mat(50,100,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im = cv::Mat(max(mIm.rows,mRefIm.rows),mIm.cols+mRefIm.cols,mIm.type());
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<cv::KeyPoint> vRefKeysUn;
    vector<int> vBTMatches;
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        // cout << "mRefIm.rows: "<<mRefIm.rows<<" mRefIm.cols: "<<mRefIm.cols<<endl;
        // cout << "mIm.rows: "<<mIm.rows<<" mIm.cols: "<<mIm.cols<<endl;
        // cout << "im.rows: "<<im.rows<<" im.cols: "<<im.cols<<endl;
        // cout <<"mRefIm type: "<<mRefIm.type()<<endl;
        // cout <<"mIm type: "<<mIm.type()<<endl;
        mIm.copyTo(im.rowRange(0,mIm.rows).colRange(0,mIm.cols));
        // im.colRange(mIm.cols,im.cols) = cv::Mat::zeros(im.rows,mRefIm.cols,im.type());
        if(mRefIm.type() == 0)
          mRefIm.copyTo(im.rowRange(0,mRefIm.rows).colRange(mIm.cols,mIm.cols+mRefIm.cols));
        else
          im.colRange(mIm.cols,im.cols) = cv::Mat::zeros(im.rows,mRefIm.cols,im.type());

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            vRefKeysUn = mvRefKeysUn;
            vBTMatches = mvBTMatches;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            vRefKeysUn = mvRefKeysUn;
            vBTMatches = mvBTMatches;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    // cout << "vBTMatches size: " << vBTMatches.size()<<", vCurrentKeys size: "<<vCurrentKeys.size()<<endl;
    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        const int nR = vRefKeysUn.size();
        for(int j=0;j<nR;j++)
        {
          cv::Point2f ptR,ptR1,ptR2;
          ptR.x=vRefKeysUn[j].pt.x+mIm.cols;
          ptR.y=vRefKeysUn[j].pt.y;
          ptR1.x=ptR.x-r-1;
          ptR1.y=ptR.y-r-1;
          ptR2.x=ptR.x+r+1;
          ptR2.y=ptR.y+r+1;
          cv::rectangle(im,ptR1,ptR2,cv::Scalar(0,253,0));
          cv::circle(im,ptR,2,cv::Scalar(0,253,0),-1);
        }
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
            else
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
                cv::rectangle(im,pt1,pt2,cv::Scalar(0,254,0));
                cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,254,0),-1);
            }
            if(vBTMatches.size()!=vCurrentKeys.size())//in case CurrentKeys update before vBTMatches update
              continue;
            if(vBTMatches[i]>=0)
            {
              cv::Point2f ptR;
              ptR.x=vRefKeysUn[vBTMatches[i]].pt.x+mIm.cols;
              ptR.y=vRefKeysUn[vBTMatches[i]].pt.y;
              cv::line(im,vCurrentKeys[i].pt,ptR,cv::Scalar(0,0,255));
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

void FrameDrawer::setSimilarity(const unsigned int &nKFload, const unsigned int &width)
{
  mSimilarityGraph = cv::Mat(nKFload,width,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawSimilarity()
{
  unique_lock<mutex> lock(mMutexSimilarity);
  cv::Mat heatmap;
  cv::applyColorMap(mSimilarityGraph, heatmap, cv::COLORMAP_JET);
  return heatmap;
}

void FrameDrawer::UpdateSimilarity(map<long unsigned int, float> vSimilarity,long unsigned int fId)
{
  unique_lock<mutex> lock(mMutexSimilarity);
  fId = fId%1000;
  int nfId = mnfId%1000;
  const float r = 1;
  cv::Point2f pt1,pt2;
  pt1.x=nfId*r;
  pt1.y=0;
  pt2.x=(nfId+1)*r-1;
  pt2.y=r-1;
  for(map<long unsigned int, float>::iterator mit=vSimilarity.begin(), mend=vSimilarity.end(); mit!=mend; mit++)
  {
    float similarity = mit->second;
    float s = pow(similarity,0.25);
    cv::rectangle(mSimilarityGraph,pt1,pt2,cv::Scalar(s*255,s*255,s*255));
    pt1.y+=r;
    pt2.y+=r;
  }
  mnfId++;
}

void FrameDrawer::UpdateBTMatch(vector<cv::KeyPoint> vRefKeysUn, vector<int> vBTMatches, cv::Mat refIm)
{
  unique_lock<mutex> lock(mMutex);
  refIm.copyTo(mRefIm);
  mvRefKeysUn = vRefKeysUn;
  mvBTMatches = vBTMatches;//store index of RefLKF's KeyPoints base on current KeyPoints
}

} //namespace ORB_SLAM

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include <pangolin/pangolin.h>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include<mutex>


class KeyFrame
{
public:
    
    cv::Mat GetWorldPos()const
    {
        return mWorldPos;
    }
    
    cv::Mat GetWorldPosInv()const
    {
        return mWorldPosInv;
    }
    
    void SetWorldPos(const cv::Mat &pos)
    {
        mWorldPos = pos.clone();
        
        cv::Mat Rcw = mWorldPos.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = mWorldPos.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        cv::Mat Ow = -Rwc*tcw;
        
        mWorldPosInv = cv::Mat::eye(4,4,pos.type());
        Rwc.copyTo(mWorldPosInv.rowRange(0,3).colRange(0,3));
        Ow.copyTo(mWorldPosInv.rowRange(0,3).col(3));
    }
    
protected:
    cv::Mat mWorldPos;
    cv::Mat mWorldPosInv;
};

class MapPoint
{
public:
    bool isBad()const
    {
        return false;
    }
    cv::Mat GetWorldPos()
    {
        return mWorldPos;
    }
    
    void SetWorldPos(const cv::Mat &pos)
    {
        mWorldPos = pos.clone();
    }
protected:
    cv::Mat  mWorldPos;
};


class Map
{
public:
    std::vector<MapPoint*> GetAllMapPoints()const
    {
        return mMpPoints;
    }
    
    std::vector<MapPoint*> GetReferenceMapPoints()const
    {
        return mMpPoints;
    }
    
    std::vector<KeyFrame*> GetAllKeyFrames()const
    {
        return mKeyFrames;
    }
    void push(MapPoint *mppt)
    {
        mMpPoints.emplace_back(mppt);
    }
    
    void push(KeyFrame *keyframe)
    {
        mKeyFrames.emplace_back(keyframe);
    }
protected:
    std::vector<MapPoint*> mMpPoints;
    std::vector<KeyFrame*> mKeyFrames;
};

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const std::string &strSettingPath);

    Map* mpMap;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
};
#endif // MAPDRAWER_H

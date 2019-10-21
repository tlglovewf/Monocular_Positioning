//
//  CFrame.h
//
//  Created by TuLigen on 2019/8/23.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _CFRAME_H_H_
#define _CFRAME_H_H_
#include <vector>
#include "opencv2/core/core.hpp"
#include "CTypes.h"
using namespace std;
using namespace cv;

#define FRAME_GRID_ROWS  48
#define FRAME_GRID_COLS  64
class Frame
{
public:

    Frame():N(0){}
    Frame(const Mat &img):mImg(img),N(0)
    {
        
    }

    
    void release()
    {
        delete this;
    }

    Mat& getDescriptor()
    {
        return mDes;
    }

    KeyPtVector& getKeys()
    {
        return mKeys;
    } 

    PtVector& getPreMatchPts()
    {
        return mPreMatchPts;
    }

    PtVector& getCurMatchPts()
    {
        return mCurMatchPts;
    }

    void setImage(const Mat &img)
    {
        mImg = img;
        mnMinx = 0.0f;
        mnMaxx = mImg.cols;
        mnMiny = 0.0f;
        mnMaxy = mImg.rows;
    }

    Mat getImage()const
    {
        return mImg;
    }

    SzVector getFeaturesInArea( float x, float y, float r, int minLevel = -1, int maxLevel = -1)const;

    void initStaticParams();

    //分配關鍵點到塊中
    void assignFeaturesToGrid();

    //判斷關鍵點在哪個grid
    inline bool PosInGrid( const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x - mnMinx) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMiny) * mfGridElementWidthInv);

        if( posX < 0 || posX >= FRAME_GRID_COLS ||
            posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;
        else
            return true;
    }
public:
    int N;
protected:
    Mat mImg;
    Mat mPose;  // 4 x 4
    Mat mDes;   //descriptor
    KeyPtVector mKeys;
    PtVector    mPreMatchPts;
    PtVector    mCurMatchPts;


    static float mnMinx;
    static float mnMaxx;
    static float mnMiny;
    static float mnMaxy;

    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;

    SzVector mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
};

class CPoint
{
public:


    Point2f getPixel()const
    {
        return mPixel;
    }

    Point3f getPt()const
    {
        return mPt;
    }

    const MFmContainer& getObservations()const
    {
        return mObservations;
    }

protected:

    Point2f     mPixel;
    Point3f     mPt;
    MFmContainer   mObservations;

};

#endif 
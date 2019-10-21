//
//  CFeatureTrack.hpp
//  MySlam
//
//  Created by TuLigen on 2019/3/12.
//

#ifndef FeatureTrack_hpp
#define FeatureTrack_hpp
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "CFrame.h"
#include "CORBextractor.h"
#include "CORBmatcher.h"

using namespace cv;

class IFeatureTrack
{
public:
    IFeatureTrack(){}
    virtual ~IFeatureTrack(){}

    virtual void calcFeatures(const Mat &img, Frame *fm) = 0;

    virtual void match( Frame *preframe, Frame *curframe, MatchVector &mtvecotr) = 0;
};

class CVFeatureTrack : public IFeatureTrack
{
public:
    CVFeatureTrack()
    {
        mOrb = ORB::create(2000, 1.2f, 8 ,31, 0, 4, ORB::HARRIS_SCORE,31,20);
    }
    
    virtual void calcFeatures(const Mat &img,Frame *fm)
    {
        assert(fm);
        mOrb->detect(img, fm->getKeys());
        mOrb->compute(img, fm->getKeys(), fm->getDescriptor());
        fm->N = fm->getKeys().size();
    }

    virtual void match( Frame *preframe, Frame *curframe, MatchVector &mtvecotr);

protected:
    Ptr<ORB> mOrb;
    Ptr<ORBextractor> mOrbExtractor;
};



class ORBFeatureTrack : public IFeatureTrack
{
public:
    ORBFeatureTrack():
    mOrbExtractor(new ORBextractor(4000,1.5,8,20,7)),
    mOrbMatcher(new ORBmatcher(0.75,true))
    {

    }
    ~ORBFeatureTrack()
    {

    }

    virtual void calcFeatures(const Mat &img, Frame *fm)
    {   
        assert(fm);
        (*mOrbExtractor)(img,cv::Mat(), fm->getKeys(), fm->getDescriptor());
        fm->N = fm->getKeys().size();
        fm->assignFeaturesToGrid();
    }
    
    virtual void match( Frame *preframe, Frame *curframe, MatchVector &mtvecotr);
protected:
    Ptr<ORBextractor> mOrbExtractor;
    Ptr<ORBmatcher>   mOrbMatcher;
};


#endif /* FeatureTrack_hpp */

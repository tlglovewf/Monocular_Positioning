//
//  CFeatureTrack.cpp
//  MySlam
//
//  Created by TuLigen on 2019/3/12.
//

#include "CFeatureTrack.h"

void CVFeatureTrack::match( Frame *preframe, Frame *curframe, MatchVector &mtvecotr)
{
    assert(preframe);
    assert(curframe);
    
    const KeyPtVector &prekeys = preframe->getKeys();
    const KeyPtVector &curkeys = curframe->getKeys();
    const Mat &descriptor1 = preframe->getDescriptor();
    const Mat &descriptor2 = curframe->getDescriptor();
    PtVector &prepts = preframe->getPreMatchPts();
    PtVector &curpts = curframe->getCurMatchPts();

    BFMatcher   mMatcher(NORM_HAMMING2);
    const float minRatio = 1.f / 1.35f ;//(4m/f)
    const int k = 2;
    
    std::vector<std::vector<DMatch> > knnMatches;
    mMatcher.knnMatch(descriptor1, descriptor2, knnMatches, k);
    
    for (size_t i = 0; i < knnMatches.size(); i++) {
        const DMatch& bestMatch = knnMatches[i][0];
        const DMatch& betterMatch = knnMatches[i][1];
        
        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < minRatio)
        {
            prepts.emplace_back(prekeys[bestMatch.queryIdx].pt);
            curpts.emplace_back(curkeys[bestMatch.trainIdx].pt);
        }
    }
    std::vector<uchar> mask;
    //  剔除外點
    //  findFundamentalMat(prepts,curpts,CV_RANSAC,3.0,0.9899,mask);//(4m/f）
    // PtVector tmppre;
    // PtVector tmpcur;
    // for( size_t i = 0; i < mask.size(); ++i)
    // {
    //     if(mask[i])
    //     {
    //         tmppre.push_back(prepts[i]);
    //         tmpcur.push_back(curpts[i]);
    //     }
    // }
    // prepts.swap(tmppre);
    // curpts.swap(tmpcur);
}

void ORBFeatureTrack::match( Frame *preframe, Frame *curframe, MatchVector &mtvecotr)
{
    assert(preframe);
    assert(curframe);
    PtVector  preKeyPtVector;
    IntVector vMatched;
    preKeyPtVector.resize(preframe->N);
    for(size_t i = 0; i < preframe->N; ++i)
    {
        preKeyPtVector[i] = preframe->getKeys()[i].pt;
    }

    mOrbMatcher->SearchForInitialization(*preframe,*curframe,preKeyPtVector, vMatched, 500);

    PtVector &prepts = preframe->getPreMatchPts();
    PtVector &curpts = curframe->getCurMatchPts();
    for(size_t i = 0; i < vMatched.size(); ++i)
    {
        if( vMatched[i] >= 0)
        {
            prepts.push_back(preframe->getKeys()[i].pt);
            curpts.push_back(curframe->getKeys()[vMatched[i]].pt);

            mtvecotr.push_back(DMatch(i,vMatched[i],0.0));
        }
        else
        {
            //add more ... 
        }
        
    }
    std::count(vMatched.begin(), vMatched.end(),-1);
    // printf("matches point %d\n", mtvecotr.size());
}
//
//  COptimizer.h
//
//  Created by TuLigen on 2019/8/23.
//  Copyright © 2019年 TuLigen. All rights reserved.
//
#ifndef _OPTIMIZER_H_H_
#define _OPTIMIZER_H_H_

#include <vector>
using namespace std;
class Frame;
class CPoint;
typedef vector<Frame*>   FrameVector;
typedef vector<CPoint*>  PointVector;
class COptimizer
{
public:

    void static BundleAdjustment(const FrameVector &frames,const PointVector &pts,const bool bRobust = true);

    
protected:

};



#endif 
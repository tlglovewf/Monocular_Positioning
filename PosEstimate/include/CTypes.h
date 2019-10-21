//
//  CTypes.h
//
//  Created by TuLigen on 2019/8/23.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _CTYPES_H_H_
#define _CTYPES_H_H_
#include <vector>
#include <deque>
#include "opencv2/core/types.hpp"

typedef std::vector<cv::KeyPoint> KeyPtVector;

typedef std::vector<cv::Point2f>  PtVector;

typedef std::vector<cv::DMatch>   MatchVector;

typedef std::vector<size_t>       SzVector;
typedef std::vector<int>          IntVector;

class CPoint;
typedef std::vector<CPoint*> MPtVector;

class Frame;
typedef std::deque<Frame*> MFmContainer;

#endif
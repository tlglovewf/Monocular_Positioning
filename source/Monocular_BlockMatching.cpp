//
//  BlockMatching.cpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/13.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include "Monocular_BlockMatching.h"

/***********************************block matching*****************************/

/*
 * 创建对象
 */
BlockMatch* BlockMatch::CreateMethod(eBlockMatchingType type,const Mat &img, const Point2f &pt)
{
    switch (type) {
        case eNCC:
            return new NCC_BlockMatch(img,pt);
            //add more
        default:
            return NULL;
    }
}

NCC_BlockMatch::NCC_BlockMatch(const Mat &mat,const Point2f &pt):_mat(mat)
{
    _mean = 0;
    // 零均值-归一化互相关
    // 先算均值
    
    Mat img;
    if(mat.channels() > 1)
    {
        cvtColor(mat, img, CV_BGR2GRAY);
    }
    else
    {
        img = mat;
    }
    for ( int x=-ncc_window_size; x<=ncc_window_size; x++ )
        for ( int y=-ncc_window_size; y<=ncc_window_size; y++ )
        {
            double value_ref = double(img.ptr<uchar>( int(y+pt.y) )[ int(x+pt.x) ]) / 255.0;
            _mean += value_ref;
            
            _values.push_back(value_ref);
            
        }
    
    _mean /= ncc_area;
    
    // 计算 Zero mean NCC
    for ( int i=0; i<_values.size(); i++ )
    {
        _demoniator += (_values[i]-_mean)*(_values[i]-_mean);
    }
}
#define CLAMP(x,a,b)  (x < a)?a : (x > b) ? b : x;
double NCC_BlockMatch::score(const Mat &cur,const Point2f &pt)
{
    // 零均值-归一化互相关
    // 先算均值
    double  mean_curr = 0;
    
    Mat img;
    if(cur.channels() > 1)
    {
        cvtColor(cur, img, CV_BGR2GRAY);
    }
    else
    {
        img = cur;
    }
    std::vector<double> values_curr; // 参考帧和当前帧的均值
    for ( int x=-ncc_window_size; x<=ncc_window_size; x++ )
        for ( int y=-ncc_window_size; y<=ncc_window_size; y++ )
        {
			Point2f ppt = pt + Point2f(x, y);

			const int len = 5;
			ppt.x = CLAMP(ppt.x, len, img.cols - len );
			ppt.y = CLAMP(ppt.y, len, img.rows - len );
            double value_curr = getBilinearInterpolatedValue( img, ppt);
            mean_curr += value_curr;
            
            values_curr.push_back(value_curr);
        }
    
    mean_curr /= ncc_area;
    
    // 计算 Zero mean NCC
    double numerator = 0, cur_demoniator = 0;
    for ( int i=0; i < _values.size(); i++ )
    {
        double n = (_values[i] - _mean) * (values_curr[i]-mean_curr);
        numerator += n;
        cur_demoniator += (values_curr[i]-mean_curr)*(values_curr[i]-mean_curr);
    }
    return numerator / sqrt( _demoniator * cur_demoniator+1e-10 );   // 防止分母出现零
}


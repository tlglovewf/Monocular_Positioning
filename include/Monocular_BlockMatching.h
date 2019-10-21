//
//  BlockMatching.hpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/13.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef BlockMatching_hpp
#define BlockMatching_hpp
#include "Monocular_Types.h"

/*
 * 块匹配算法
 */
enum eBlockMatchingType
{
    eNCC,
    eSAD,
    eSSD
};

//块匹配
class BlockMatch
{
public:
    /*
     * 块评分
     * @param cur 当前帧
     * @param pt  像素点
     * @return    评分
     */
    virtual double score(const Mat &cur,const Point2f &pt) = 0;
    
    /*
     * 创建对象
     */
    static BlockMatch* CreateMethod(eBlockMatchingType type,const Mat &img, const Point2f &pt);
};
//NCC 算法
class NCC_BlockMatch : public BlockMatch
{
    const int ncc_window_size = 4;    // NCC 取的窗口半宽度
    const int ncc_area = (2*ncc_window_size+1)*(2*ncc_window_size+1); // NCC窗口面积
public:
    /*
     * 构造函数
     */
    NCC_BlockMatch(const Mat &mat,const Point2f &pt);
    
    /*
     * 块评分
     * @param cur 当前帧
     * @param pt  像素点
     * @return    评分
     */
    virtual double score(const Mat &cur,const Point2f &pt);
protected:
    // 双线性灰度插值
    inline double getBilinearInterpolatedValue( const Mat& img, const Point2d& pt ) {
        uchar* d = & img.data[ int(pt.y)*img.step+int(pt.x) ];
        double xx = pt.x - floor(pt.x);
        double yy = pt.y - floor(pt.y);
        return  (( 1-xx ) * ( 1-yy ) * double(d[0]) +
                 xx* ( 1-yy ) * double(d[1]) +
                 ( 1-xx ) *yy* double(d[img.step]) +
                 xx*yy*double(d[img.step+1]))/255.0;
    }
protected:
    const Mat&           _mat;
    double               _mean;
    std::vector<double>  _values;
    double               _demoniator;
};

//SSD
class SSD_BlockMatch : public BlockMatch
{
public:
    /*
     * 块评分
     * @param cur 当前帧
     * @param pt  像素点
     * @return    评分
     */
    virtual double score(const Mat &cur,const Point2f &pt) {return 0.0;}
};

//SAD
class SAD_BlockMatch : public BlockMatch
{
public:
    /*
     * 块评分
     * @param cur 当前帧
     * @param pt  像素点
     * @return    评分
     */
    virtual double score(const Mat &cur,const Point2f &pt) {return 0.0;}
};

#endif /* BlockMatching_hpp */

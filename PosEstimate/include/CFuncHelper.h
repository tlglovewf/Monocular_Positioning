//
//  CFuncHelper.h
//
//  Created by TuLigen on 2019/8/9.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef FuncHelper_hpp
#define FuncHelper_hpp

#include <vector>
#include <string>
#include <fstream>
#include <opencv2/core/types.hpp>
#include "assert.h"
using namespace std;

typedef std::vector<string>     FileNameVec;
typedef FileNameVec::iterator   FileNamVIter;

extern bool isPicSuffix(const char *pName,size_t len);

extern int loadFiles( const std::string &dirpath, FileNameVec &files );

/*  write real trace
 */
extern bool writeRealTrace(ofstream &os,const BLHCoordinate &blh,const std::string &filename);
/* write estimate trace
 */
extern bool writeEstTrace(ofstream &os,const BLHCoordinate &blh, const cv::Point3d &res, const std::string &filename);

#endif /* FuncHelper_hpp */

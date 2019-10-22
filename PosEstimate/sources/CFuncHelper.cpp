//
//  CFuncHelper.cpp
//  SlamToTruth
//
//  Created by TuLigen on 2019/8/9.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include "Monocular_Utils.h"
#include "CFuncHelper.h"
#include <strings.h>
#include <string.h>
#include "dirent.h"
bool isPicSuffix(const char *pName,size_t len)
{
    const size_t suffix = 3;
    assert(pName);
    if(len < suffix)
    {
        return false;
    }
    const char *pSuffix =  &pName[len - suffix];
    
    return !strcasecmp(pSuffix, "jpg") | !strcasecmp(pSuffix, "png");
}
int loadFiles( const std::string &dirpath, FileNameVec &files )
{
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dirpath.c_str())) == NULL)
    {
        assert(NULL);
    }
    int index = 0;
    while((dirp = readdir(dp)) != NULL)
    {
        if(isPicSuffix(dirp->d_name,strlen(dirp->d_name)))
        {
            std::string filepath(dirpath);
            filepath.append("/");
            filepath.append(dirp->d_name);
            files.emplace_back(filepath);
            // printf("%s\n",filepath.c_str());
            ++index;
        }
    }
    closedir(dp);
    sort(files.begin(),files.end());
    return index;
}

/* write real trace
 */
bool writeRealTrace(ofstream &os,const BLHCoordinate &blh,const std::string &filename)
{
    os << blh.latitude << "," << blh.longitude << "," << filename.c_str() << endl;
    return true;
}
/* write estimate trace
 */
bool writeEstTrace(ofstream &os,const BLHCoordinate &blh, const cv::Point3d &res, const std::string &filename)
{
    os << blh.latitude << "," << blh.longitude << "," << res.x << "," << res.y << "," << res.z <<
            "," << filename.c_str() << endl;
    return true;
}
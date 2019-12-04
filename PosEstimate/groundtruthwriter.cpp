//
//  main.cpp
//  PosEstimate
//
//  Created by TuLigen on 2019/8/9.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include <fstream>
#include <algorithm>
#include "M_Utils.h"
#include "CFuncHelper.h"
#include "M_CoorTransform.h"
#include "M_Config.h"
#include "M_DataLoader.h"
#include "CFeatureTrack.h"
#include <thread>
#include "algorithm"

#include "CDataManager.h"

using namespace cv;
using namespace std;

#define FTLEN 25
#define WRITESPACE(FILE) FILE.width(FTLEN);
#define WRITEPOSEDATA(FILE, NAME, TIME, LON, LAT, ALT, PITCH, YAW, ROLL, V) \
    FILE.width(FTLEN * 2);                                               \
    FILE << NAME;                                                        \
    WRITESPACE(FILE)                                                     \
    FILE << TIME;                                                        \
    WRITESPACE(FILE)                                                     \
    FILE << LON;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << LAT;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << ALT;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << PITCH;                                                       \
    WRITESPACE(FILE)                                                     \
    FILE << YAW;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << ROLL;                                                        \
    WRITESPACE(FILE)                                                     \
    FILE << V;                                                           \
    FILE << endl;

#define WRITEIMURAWDATA(FILE, TIME, GX, GY, GZ, AX, AY, AZ) \
    WRITESPACE(FILE);                                       \
    FILE << TIME;                                           \
    WRITESPACE(FILE);                                       \
    FILE << GX;                                             \
    WRITESPACE(FILE);                                       \
    FILE << GY;                                             \
    WRITESPACE(FILE);                                       \
    FILE << GZ;                                             \
    WRITESPACE(FILE);                                       \
    FILE << AX;                                             \
    WRITESPACE(FILE);                                       \
    FILE << AY;                                             \
    WRITESPACE(FILE);                                       \
    FILE << AZ;                                             \
    FILE << endl;



// main func
int main(int argc, const char *argv[])
{
    if(argc < 3)
    {
        cout << "please input right file path ." << endl;
        cout << "input processed image path  and output file path." << endl;
        return -1;
    }
    cout.precision(20);

    CDataManager::getSingleton()->LoadData(string(argv[1]) + "/pst.txt",string(argv[1]) + "/imu.txt");

    ImgInfoVIter bgIter = CDataManager::getSingleton()->begin();
    ImgInfoVIter edIter = CDataManager::getSingleton()->end();

    ofstream realfile;

    realfile.open(argv[2]);
    realfile.precision(15);

    CDataManager::getSingleton()->setIndicator(0);
    cout << "begin write file." << endl;
    for(; bgIter != edIter; ++bgIter)
    {
        writeRealTrace(realfile,bgIter->second.pos,bgIter->first.substr(28,4));
        {
            cout << "------------------" << bgIter->second.pos.longitude << " " << bgIter->second.pos.latitude << endl;
        }
    }
    realfile.close();
    cout << "write successfully." << endl;
    return 0;
}

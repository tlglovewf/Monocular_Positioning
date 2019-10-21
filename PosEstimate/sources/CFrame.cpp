#include "CFrame.h"

float Frame::mnMinx = 0.0f;
float Frame::mnMaxx = 0.0f;
float Frame::mnMiny = 0.0f;
float Frame::mnMaxy = 0.0f;

float Frame::mfGridElementWidthInv  = 0.0f;
float Frame::mfGridElementHeightInv = 0.0f;

void Frame::initStaticParams()
{
    //calc bound 
    mnMinx = 0.0f;
    mnMaxx = mImg.cols;
    mnMiny = 0.0f;
    mnMaxy = mImg.rows;

    //calc other params
    mfGridElementWidthInv  = static_cast<float>(FRAME_GRID_COLS) / (mnMaxx - mnMinx);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxy - mnMiny);

    //add more ...
}

void Frame::assignFeaturesToGrid()
{
    int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
    for( int i = 0; i < FRAME_GRID_COLS; ++i)
    {
        for( int j = 0; j < FRAME_GRID_ROWS; ++j)
        {
            mGrid[i][j].reserve(nReserve);
        }
    }

    for(int i = 0;i < N; ++i)
    {
        const cv::KeyPoint &kp = mKeys[i];

        int nGridPosX, nGridPoxY;
        if( PosInGrid(kp, nGridPosX, nGridPoxY))
        {
            mGrid[nGridPosX][nGridPoxY].push_back(i);
        }
    }
}

SzVector Frame::getFeaturesInArea( float x, float y, float r, 
                                   int minLevel /* = -1 */, int maxLevel /* = -1 */)const
{
    SzVector vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0, (int)floor((x - mnMinx - r) * mfGridElementWidthInv));
    if(nMinCellX >= FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS - 1,(int)ceil((x - mnMinx + r) * mfGridElementWidthInv));
    if(nMaxCellX < 0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y - mnMiny - r) * mfGridElementHeightInv));
    if(nMinCellY >= FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1,(int)ceil((y - mnMiny + r) * mfGridElementHeightInv));
    if(nMaxCellY < 0)
        return vIndices;

    const bool bCheckLevels = ( minLevel > 0 ) || ( maxLevel >= 0 ) ;

    for(int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend = vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mKeys[ vCell[j] ];
                if(bCheckLevels)
                {
                    if(kpUn.octave < minLevel)
                        continue;
                    if(maxLevel >= 0)
                        if(kpUn.octave > maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                // if(fabs(distx)<r && fabs(disty)<r)
                //     vIndices.push_back(vCell[j]);

                float d = sqrt( distx * distx + disty * disty);

                if(d < r)
                {
                    vIndices.push_back(vCell[j]);
                }

            }
        }
    }

    return vIndices;
}
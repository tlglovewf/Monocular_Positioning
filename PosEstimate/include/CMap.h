#ifndef _CMAP_H_H_
#define _CMAP_H_H_
#include "CTypes.h"
class Frame;

class CMap
{
public:
    /*
     *  local window size 
     */
    const size_t LocalWinSize = 10;
    void push(Frame *frame)
    {
        mFms.push_back(frame);
        if( mFms.size () > LocalWinSize )
        {
            mFms.front()->release();
            mFms.pop_front();
        }
    }

    const MFmContainer& getFrames()const
    {
        return mFms;
    }

protected:
    MFmContainer mFms;
};

#endif 
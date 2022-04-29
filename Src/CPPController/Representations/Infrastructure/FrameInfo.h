#pragma once

class FrameInfo
{
public:
    int getTimeSince(unsigned timeStamp) const;
    unsigned time = 0;
};

inline int FrameInfo::getTimeSince(unsigned timeStamp) const
{
    return static_cast<int>(time - timeStamp);
}
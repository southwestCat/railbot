#pragma once

#include "point_3d.h"

struct YPR {
    float yaw, pitch, roll;

    YPR():yaw(0),pitch(0),roll(0){}
    YPR(float _yaw,float _pitch,float _roll):yaw(_yaw),pitch(_pitch),roll(_roll){}

    YPR operator-(const YPR &o){
        return YPR(yaw-o.yaw,pitch-o.pitch,roll-o.roll);
    }

    YPR operator+(const YPR &o){
        return YPR(yaw+o.yaw,pitch+o.pitch,roll+o.roll);
    }
};

struct BodyAngles
{
    float X;
    float Y;
    float Z;
};

struct IMU {
    YPR gyr;
    point_3d accel;
    BodyAngles angles;
};

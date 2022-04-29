#pragma once

struct HeadTouch
{
    float front;
    float middle;
    float rear;
};

struct LFootBumper
{
    float left;
    float right;
};

struct RFootBumper
{
    float left;
    float right;
};

struct LHandTouch
{
    float back;
    float left;
    float right;
};

struct RHandTouch
{
    float back;
    float left;
    float right;
};

struct Touch
{
    float ChestBtn;
    HeadTouch headTouch;
    LFootBumper lFootBumper;
    RFootBumper rFootBumper;
    LHandTouch lHandTouch;
    RHandTouch rHandTouch;
};
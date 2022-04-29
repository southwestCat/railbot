#pragma once

#include <array>

struct RGB {
    float r, g, b;
    static const RGB RED;
    static const RGB GREEN;
    static const RGB BLUE;
};

using Eye = std::array<RGB, 8>;

struct Eyes {
    Eye left;
    Eye right;
};

struct Ears {
    std::array<float, 10> left;
    std::array<float, 10> right;
};

struct Foots {
    RGB left;
    RGB right;
};

struct Leds {
    Ears ears;
    Eyes eyes;
    RGB chestLed;
    Foots foots;
    std::array<float, 12> head;
};
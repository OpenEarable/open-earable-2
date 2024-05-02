#ifndef SOSFilter_H
#define SOSFilter_H

#define EQ_ORDER 1
//3

#include <zephyr/kernel.h>

class SOSFilter {
public:
    SOSFilter();
    ~SOSFilter();

    void reset();
    void update(int32_t * data, int length);
private:
    float buffer[EQ_ORDER][2] = {0};

    //const float b[EQ_ORDER][3] = {{0.13110644,  0.26221288, 0.13110644}};
    //const float a[EQ_ORDER][3] = {{1         , -0.74778918, 0.27221494}};

    const float b[EQ_ORDER][3] = { 0.0674552738890719, 0.1349105477781438, 0.0674552738890719 };
    const float a[EQ_ORDER][3] = { 1.0,                -1.1429805025399011, 0.4128015980961887 };
};

#endif

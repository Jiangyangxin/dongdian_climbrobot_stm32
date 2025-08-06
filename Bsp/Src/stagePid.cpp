#include "stagePid.h"
#include "math.h"

#define USEI
#define USED

StagePid::StagePid(float p1, float i1, float d1, float p2, float i2, float d2, float n, float ts, float diffLimit, float valLimit, float outIMin, float outIMax, float outMin, float outMax)
{
    this->p1 = p1;
    this->i1 = i1;
    this->d1 = d1;
    this->p2 = p2;
    this->i2 = i2;
    this->d2 = d2;
    this->n = n;
    this->ts = ts;
    this->diffLimit = diffLimit;
    this->valLimit = valLimit;
    this->accI = .0f;
    this->accD = .0f;
    this->accIMin = outIMin;
    this->accIMax = outIMax;
    this->accDMin = outMin / n;
    this->accDMax = outMax / n;
    this->outMax = outMax;
    this->outMin = outMin;
}

void StagePid::SetParam(float p1, float i1, float d1, float p2, float i2, float d2)
{
    this->p1 = p1;
    this->i1 = i1;
    this->d1 = d1;
    this->p2 = p2;
    this->i2 = i2;
    this->d2 = d2;
    this->accI = .0f;
    this->accD = .0f;
}

void StagePid::Reset()
{
    this->accI = .0f;
    this->accD = .0f;
}

float StagePid::Tick(float diff, float cur)
{
    float pout;
    float p, i, d;
#ifdef USED
    float dout;
#endif
// first stage deadzone
    if (fabs(diff) < diffLimit)
    {
        pout = 0.f;
        return pout;
    }
    if (fabs(cur) > valLimit)
    {
        p = p2;
        i = i2;
        d = d2;
        accI = .0f;
        accD = .0f;
    }
    else
    {
        p = p1;
        i = i1;
        d = d1;
        accI = .0f;
        accD = .0f;
    }

    pout = diff * p;

#ifdef USEI
    accI += diff * i * ts;
    if (accI > accIMax)
        accI = accIMax;
    else if (accI < accIMin)
        accI = accIMin;
#endif

#ifdef USED
    dout = (diff * d - accD) * n;
    accD += dout * ts;
    if (accD > accDMax)
        accD = accDMax;
    else if (accD < accDMin)
        accD = accDMin;
#endif

#ifdef USEI
    pout += accI;
#endif
#ifdef USED
    pout += dout;
#endif

    if (pout > outMax)
        pout = outMax;
    else if (pout < outMin)
        pout = outMin;

    return pout;
}

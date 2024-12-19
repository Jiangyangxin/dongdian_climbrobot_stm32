#include "deadZonePid.h"
#include "math.h"

#define USEI
#define USED

DeadZonePid::DeadZonePid(float p, float i, float d, float n, float ts, float limit, float outIMin, float outIMax, float outMin, float outMax)
{
    this->p = p;
    this->i = i;
    this->d = d;
    this->n = n;
    this->ts = ts;
    this->limit = limit;
    this->accI = .0f;
    this->accD = .0f;
    this->accIMin = outIMin;
    this->accIMax = outIMax;
    this->accDMin = outMin / n;
    this->accDMax = outMax / n;
    this->outMax = outMax;
    this->outMin = outMin;
}

void DeadZonePid::SetParam(float p, float i, float d)
{
    this->p = p;
    this->i = i;
    this->d = d;
    this->accI = .0f;
    this->accD = .0f;
}

void DeadZonePid::Reset()
{
    this->accI = .0f;
    this->accD = .0f;
}

float DeadZonePid::Tick(float diff)
{
    float pout;
#ifdef USED
    float dout;
#endif

    pout = diff * p;

#ifdef USEI
    if (fabs(diff) > limit)
    {
        accI += diff * i * ts;
        if (accI > accIMax)
            accI = accIMax;
        else if (accI < accIMin)
            accI = accIMin;
    }
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

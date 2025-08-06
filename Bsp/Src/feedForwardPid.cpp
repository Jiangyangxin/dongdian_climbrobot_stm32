#include "feedForwardPid.h"
#include "math.h"
#include "physparams.h"

float midfilter(float in)
{
    static float midbuffer[4] = {0};
    midbuffer[0] = in;
    for (int i = 3; i > 0; i--)
    {
        midbuffer[i] = midbuffer[i - 1];
    }

    // Direct access to elements indexed 1 to 3
    float a = midbuffer[1];
    float b = midbuffer[2];
    float c = midbuffer[3];

    float median;

    // find median
    if ((a >= b && a <= c) || (a >= c && a <= b))
    {
        median = a;
    }
    else if ((b >= a && b <= c) || (b >= c && b <= a))
    {
        median = b;
    }
    else
    {
        median = c;
    }

    return median;
}

float midfilter4(float in)
{
    static float midbuffer[4] = {0};

    // update array, place new input in the first index
    for (int i = 3; i > 0; i--)
    {
        midbuffer[i] = midbuffer[i - 1];
    }
    midbuffer[0] = in;

    // copy value to temp array for sorting
    float temp[4];
    for (int i = 0; i < 4; i++)
    {
        temp[i] = midbuffer[i];
    }

    // bubbling sorting
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3 - i; j++)
        {
            if (temp[j] > temp[j + 1])
            {
                float t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }

    // Calculate the median
    float median = (temp[1] + temp[2]) / 2.0;

    return median;
}

ffPid::ffPid(float p, float i, float wc, float ts, float outIMin, float outIMax, float Iband, float outMin, float outMax)
{
    this->p = p;
    this->i = i;
    this->wc = wc;
    this->ts = ts;
    this->accI = .0f;
    this->accIMin = outIMin;
    this->accIMax = outIMax;
    this->Iband = Iband; // integral separated
    this->outMax = outMax;
    this->outMin = outMin;
    this->pre_tar = 0;
    this->pre_tar_dot_filtered = 0;
    this->tar_dot = 0;
}

void ffPid::Reset()
{
    this->accI = .0f;
    this->pre_tar_dot_filtered = 0.f;
}

float ffPid::Tick(float diff, float tar_dot)
{
    // PI
    float pout;

    pout = diff * p;

    if (abs(diff) < Iband)
    {
        accI += diff * i * ts;
        if (accI > accIMax)
            accI = accIMax;
        else if (accI < accIMin)
            accI = accIMin;
    }
    else
    {
        accI = 0;
    }

    pout += accI;

    // Feedforward
    // Low-pass filter
    // saturate delta value
    if (abs(tar_dot) > maxSteerOmg)
    {
        tar_dot = tar_dot > 0 ? maxSteerOmg : -maxSteerOmg;
    }

    // Low-pass filter
    float tar_dot_filtered = 0;
    tar_dot_filtered = (1 - wc * ts) * pre_tar_dot_filtered + ts * wc * tar_dot;

    if (tar_dot_filtered > maxSteerOmg)
    {
        tar_dot_filtered = maxSteerOmg;
    }
    else if (tar_dot_filtered < -maxSteerOmg)
    {
        tar_dot_filtered = -maxSteerOmg;
    }
    pre_tar_dot_filtered = tar_dot_filtered;

    pout += tar_dot_filtered;

    if (pout > outMax)
        pout = outMax;
    else if (pout < outMin)
        pout = outMin;

    return pout;
}

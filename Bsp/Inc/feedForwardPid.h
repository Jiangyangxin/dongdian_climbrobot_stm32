#ifndef __FEEDFORWARDPID_H__
#define __FEEDFORWARDPID_H__

#ifdef __cplusplus
class ffPid
{
public:
    float p, i, wc, ts, pre_tar, pre_tar_dot_filtered, tar_dot;
    float accI, accIMax, accIMin, outMax, outMin, Iband;

public:
    ffPid(float p, float i, float wc, float ts, float outIMin, float outIMax, float Iband, float outMin, float outMax);
    float Tick(float diff, float tar);
    void Reset();
};
#endif
#endif

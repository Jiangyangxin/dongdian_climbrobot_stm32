#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
class Pid
{
private:
    float p, i, d, n, ts;
    float accI, accD, accIMax, accIMin, accDMax, accDMin, outMax, outMin, Iband;
public:
    Pid(float p, float i, float d, float n, float ts,  float outIMin, float outIMax, float Iband, float outMin, float outMax);
    float Tick(float diff);
    void Reset();
    void SetParam(float p, float i);
};
#endif
#endif

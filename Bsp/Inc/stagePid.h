#ifndef __DEADZONE_PID_H__
#define __DEADZONE_PID_H__

#ifdef __cplusplus
class StagePid
{
private:
    float p1, i1, d1, p2, i2, d2, n, ts, diffLimit, valLimit;
    float accI, accD, accIMax, accIMin, accDMax, accDMin, outMax, outMin;

public:
    StagePid(float p1, float i1, float d1, float p2, float i2, float d2, float n, float ts, float diffLimit, float valLimit, float outIMin, float outIMax, float outMin, float outMax);
    float Tick(float diff, float cur);
    void Reset();
    void SetParam(float p1, float i1, float d1, float p2, float i2, float d2);
};
#endif /* __cplusplus*/
#endif /* __DEADZONE_PID_H__*/

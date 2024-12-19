#ifndef __DEADZONE_PID_H__
#define __DEADZONE_PID_H__

#ifdef __cplusplus
class DeadZonePid
{
private:
    float p, i, d, n, ts, limit;
    float accI, accD, accIMax, accIMin, accDMax, accDMin, outMax, outMin;

public:
    DeadZonePid(float p, float i, float d, float n, float ts, float limit, float outIMin, float outIMax, float outMin, float outMax);
    float Tick(float diff);
    void Reset();
    void SetParam(float p, float i, float d);
};
#endif /* __cplusplus*/
#endif /* __DEADZONE_PID_H__*/

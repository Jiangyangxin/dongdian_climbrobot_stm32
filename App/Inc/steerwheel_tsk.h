
#ifndef __STEERWHEEL_TSK_H_
#define __STEERWHEEL_TSK_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "pid.h"
#include "feedForwardPid.h"
#include "physparams.h"

namespace TskSteer
{
    typedef enum
    {
        STOP,
        NORMAL,
        TORQUE,
        RESET,
        RESETTING,
        RESETOVER,
    } steerState;

    void Init();
};

#endif /* __cplusplus */
#endif /* __STEERWHEEL_TSK_H_ */
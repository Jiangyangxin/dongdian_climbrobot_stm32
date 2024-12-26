/*
 * adsorption task, only for motor
 */

#ifndef __ADSORPTION_MOTION_TSK_H_
#define __ADSORPTION_MOTION_TSK_H_

#define ADC_CONV_FINISH_VALUE 0x1000

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
#include "stagePid.h"
#include "physparams.h"

namespace TskMotion
{
    typedef enum
    {
        STOP,
        START,
        MANUAL,
        RESET,
        RESETTING,
        FINDZERO,
        RESETOVER,
        INIT,
    } motionState;
    typedef enum
    {
        MOTOR,
        SENSOR,
    } displacementSrc;
    void Init();
};

#endif /* __cplusplus */
#endif /* __ADSORPTION_MOTION_TSK_H_*/
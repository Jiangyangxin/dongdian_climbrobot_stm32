/*
 * adsorption task, only for fan
 */

#ifndef __ADSORPTION_FAN_TSK_H_
#define __ADSORPTION_FAN_TSK_H_

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
#include "physparams.h"

namespace TskFan
{
    typedef enum
    {
        STOP,
        START,
        VACZERO,
        VACZEROCOMPLETE
    } fanState;
    void Init();
};

#endif /* __cplusplus */
#endif /* __ADSORPTION_FAN_TSK_H_*/
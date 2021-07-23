/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mixer_v22_osprey.cpp
 *
 * V22 Osprey mixers.
 */
#include <px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <math.h>

#include <px4iofirmware/protocol.h>

#include "mixer.h"


V22OspreyMixer::V22OspreyMixer( ControlCallback control_cb, uintptr_t cb_handle,
                                uint8_t setupFlagA,
                                float *rollVolCurveA,
                                float *foreAftVolCurveA,
                                float *yawVolCurveA,
                                float colVolumeA,
                                float *leftColCurveA,
                                float *rightColCurveA,
                                V22OspreyServo servo_dataA[NUM_MAIN_SERVOS]) :
    Mixer(control_cb, cb_handle),
    setup_flag(setupFlagA),
    colVolume(colVolumeA)
{
    uint32_t iL;

    for(iL=0; iL<NUM_VOL_POINTS; iL++)
    {
        rollVolCurve[iL] = rollVolCurveA[iL];
        foreAftVolCurve[iL] = foreAftVolCurveA[iL];
        yawVolCurve[iL] = yawVolCurveA[iL];
    }

    for(iL=0; iL<NUM_COL_POINTS; iL++)
    {
        leftColCurve[iL] = leftColCurveA[iL];
        rightColCurve[iL] = rightColCurveA[iL];
    }

    for(iL=0; iL<NUM_MAIN_SERVOS; iL++)
        servos[iL] = servo_dataA[iL];
}



V22OspreyMixer::~V22OspreyMixer()
{
}



float V22OspreyMixer::constrain(float val, float min, float max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}



float V22OspreyMixer::computeValueFromCurve(float pointA, float minRangeA, float maxRangeA, float *curveA, uint32_t numPointsA)
{
    float rangeA = maxRangeA - minRangeA;
    float deltaL = rangeA / (numPointsA -1);
    uint32_t idL = (pointA - minRangeA) / deltaL;

    if(idL > numPointsA - 2)
        idL = numPointsA - 2;

    return (curveA[idL + 1] - curveA[idL]) / deltaL * (pointA - minRangeA - deltaL * idL) + curveA[idL];
}




V22OspreyMixer *
V22OspreyMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
    V22OspreyServo servosL[NUM_MAIN_SERVOS];
    float forVolL[NUM_VOL_POINTS];
    float rollVolL[NUM_VOL_POINTS];
    float yawVolL[NUM_VOL_POINTS];
    float colVolL;
    float leftColCurveL[NUM_COL_POINTS];
    float rightColCurveL[NUM_COL_POINTS];
    uint32_t volL[NUM_VOL_POINTS];
    int32_t colL[NUM_COL_POINTS];
    uint32_t idL;
    int32_t minL, centerL, maxL;
    uint32_t iL;
    uint32_t setupFlagL = 0;
    float divisorL = 10000.0f;

	/* enforce that the mixer ends with a new line */
	if (!string_well_formed(buf, buflen)) {
		return nullptr;
	}

    //  GET THE VOLUMES FOR THE MIXER
    if (sscanf(buf, "A:%u%u", &setupFlagL, &volL[0]) != 2)
    {
        return nullptr;
    }

    buf = skipline(buf, buflen);

    if (buf == nullptr)
    {
        return nullptr;
    }
    else
    {
        colVolL = volL[0] / divisorL;
    }

    //  GET THE ROLL VOLUME CURVE
    buf = findtag(buf, buflen, 'W');

    if (sscanf(buf, "W:%u%u%u", &volL[0], &volL[1], &volL[2]) != NUM_VOL_POINTS)
    {
        return nullptr;
    }

    buf = skipline(buf, buflen);

    if(buf == nullptr)
    {
        return nullptr;
    }
    else
    {
        for(iL=0; iL<NUM_VOL_POINTS; iL++)
        {
            rollVolL[iL] = volL[iL] / divisorL;
        }
    }

    //  GET THE FOR/AFT VOLUME CURVE
    buf = findtag(buf, buflen, 'F');

    if (sscanf(buf, "F:%u%u%u", &volL[0], &volL[1], &volL[2]) != NUM_VOL_POINTS)
    {
        return nullptr;
    }

    buf = skipline(buf, buflen);

    if(buf == nullptr)
    {
        return nullptr;
    }
    else
    {
        for(iL=0; iL<NUM_VOL_POINTS; iL++)
        {
            forVolL[iL] = volL[iL] / divisorL;
        }
    }

    //  GET THE YAW VOLUME CURVE
    buf = findtag(buf, buflen, 'Y');

    if (sscanf(buf, "Y:%u%u%u", &volL[0], &volL[1], &volL[2]) != NUM_VOL_POINTS)
    {
        return nullptr;
    }

    buf = skipline(buf, buflen);

    if(buf == nullptr)
    {
        return nullptr;
    }
    else
    {
        for(iL=0; iL<NUM_VOL_POINTS; iL++)
        {
            yawVolL[iL] = volL[iL] / divisorL;
        }
    }

    //  GET THE LEFT NACELLE COLLECTIVE CURVE
    buf = findtag(buf, buflen, 'P');

    if (sscanf(buf, "P:%d%d%d%d%d", &colL[0], &colL[1], &colL[2], &colL[3], &colL[4]) != NUM_COL_POINTS)
    {
        return nullptr;
    }

    buf = skipline(buf, buflen);

    if(buf == nullptr)
    {
        return nullptr;
    }
    else
    {
        for(iL=0; iL<NUM_COL_POINTS; iL++)
        {
            leftColCurveL[iL] = colL[iL] / divisorL;
        }
    }

    //  GET THE RIGHT NACELLE COLLECTIVE CURVE
    buf = findtag(buf, buflen, 'Q');

    if (sscanf(buf, "Q:%d%d%d%d%d", &colL[0], &colL[1], &colL[2], &colL[3], &colL[4]) != NUM_COL_POINTS)
    {
        return nullptr;
    }

    buf = skipline(buf, buflen);

    if (buf == nullptr)
    {
        return nullptr;
    }
    else
    {
        for (iL=0; iL<NUM_COL_POINTS; iL++)
        {
            rightColCurveL[iL] = colL[iL] / divisorL;
        }
    }

    //  GET THE SERVO DATA
    for (iL=0; iL<NUM_MAIN_SERVOS; iL++)
    {
        buf = findtag(buf, buflen, 'X');

        if (sscanf(buf, "X:%d%d%d%d", &idL, &minL, &centerL, &maxL) != 4)
        {
            return nullptr;
        }

        buf = skipline(buf, buflen);

        if(buf == nullptr)
        {
            return nullptr;
        }
        else
        {
            servosL[idL].min = minL / divisorL;
            servosL[idL].center = centerL / divisorL;
            servosL[idL].max = maxL / divisorL;
        }
    }

    V22OspreyMixer *ospreyL = new V22OspreyMixer(control_cb,
                                                 cb_handle,
                                                 (uint8_t)setupFlagL,
                                                 rollVolL,
                                                 forVolL,
                                                 yawVolL,
                                                 colVolL,
                                                 leftColCurveL,
                                                 rightColCurveL,
                                                 servosL);

    return ospreyL;
}



unsigned V22OspreyMixer::mix(float *outputs, unsigned space)
{
    uint32_t iL;

    float tempL;
    float ctrlL[NUM_CONTROLS];
    float nacelleL;
    float col_valueL;

    if(space < NUM_MAIN_SERVOS)
        return 0;

    //  GET THE CONTROL DATA AND ZERO THE OUTPUTS
    if(setup_flag)
    {
        ctrlL[AIL] = get_control(3, AIL);
        ctrlL[ELEV] = get_control(3, ELEV);
        ctrlL[RUD] = get_control(3, RUD);
    }
    else
    {
        ctrlL[AIL] = get_control(0, AIL);
        ctrlL[ELEV] = get_control(0, ELEV);
        ctrlL[RUD] = get_control(0, RUD);
    }

    //ctrlL[THR] = get_control(3, 3);
    //ctrlL[COL] = get_control(0, THR);
    ctrlL[THR] = get_control(0, THR);
    ctrlL[COL] = get_control(3, 3);
    ctrlL[NACELLE] = -get_control(3, 5);

    //  CHANGE THE NACELLE TO 0->1 RANGE FOR COMPUTATIONS
    nacelleL = (ctrlL[NACELLE] + 1.0f) / 2.0f;

    //  CHANGE THE NACELLE TO SINE WAVE
    nacelleL = (1.0f + cosf(M_PI_F * nacelleL - M_PI_F)) / 2.0f;

    for(iL=0; iL<NUM_MAIN_SERVOS; iL++)
        outputs[iL] = 0.0f;

    //  COMPUTE FORE/AFT
    tempL = ctrlL[ELEV] * computeValueFromCurve(nacelleL, 0.0f, 1.0f, foreAftVolCurve, NUM_VOL_POINTS);

    outputs[LEFT_S0] -= tempL;
    outputs[LEFT_S1] += tempL;

    outputs[RIGHT_S0] -= tempL;
    outputs[RIGHT_S1] += tempL;

    //  COMPUTE ROLL COLLECTIVE COMPONENT
    tempL = ctrlL[AIL] * (1.0f - nacelleL) * computeValueFromCurve(nacelleL, 0.0f, 1.0f, rollVolCurve, NUM_VOL_POINTS);

    outputs[LEFT_S0] -= tempL;
    outputs[LEFT_S1] -= tempL;

    outputs[RIGHT_S0] += tempL;
    outputs[RIGHT_S1] += tempL;

    //  COMPUTE ROLL FORE AFT COMPONENT
    tempL = ctrlL[AIL] * nacelleL * computeValueFromCurve(nacelleL, 0.0f, 1.0f, rollVolCurve, NUM_VOL_POINTS);

    outputs[LEFT_S0] -= tempL;
    outputs[LEFT_S1] += tempL;

    outputs[RIGHT_S0] += tempL;
    outputs[RIGHT_S1] -= tempL;

    //  COMPUTE YAW FORE AFT COMPONENT
    tempL = ctrlL[RUD] * (1.0f - nacelleL) * computeValueFromCurve(nacelleL, 0.0f, 1.0f, yawVolCurve, NUM_VOL_POINTS);

    outputs[LEFT_S0] += tempL;
    outputs[LEFT_S1] -= tempL;

    outputs[RIGHT_S0] -= tempL;
    outputs[RIGHT_S1] += tempL;

    //  COMPUTE YAW COLLECTIVE COMPONENT
    tempL = ctrlL[RUD] * nacelleL * computeValueFromCurve(nacelleL, 0.0f, 1.0f, yawVolCurve, NUM_VOL_POINTS);

    outputs[LEFT_S0] -= tempL;
    outputs[LEFT_S1] -= tempL;

    tempL = ctrlL[RUD] * nacelleL * computeValueFromCurve(nacelleL, 0.0f, 1.0f, yawVolCurve, NUM_VOL_POINTS);

    outputs[RIGHT_S0] += tempL;
    outputs[RIGHT_S1] += tempL;

    //  CHANGE THROTTLE TO -1 --> 1 RANGE FOR OUTPUT
    ctrlL[THR] = ctrlL[THR] * 2.0f - 1.0f;
    outputs[LEFT_ST] = ctrlL[THR];
    outputs[RIGHT_ST] = ctrlL[THR];

    //  CONVERT THRUST TO -1 --> 1 RANGE
    //ctrlL[COL] = ctrlL[THR] * 2.0f - 1.0f;
    //ctrlL[COL] = ctrlL[COL] * 2.0f - 1.0f;

    //  COMPUTE COLLECTIVE
    if(setup_flag)
        //  GIVES FULL RANGE OF COLLECTIVE FOR SETUP PURPOSES
        col_valueL = ctrlL[COL];
    else
        //  THIS COMPRESSES THE RANGE TO 75% AND THEN ADDS BASED ON NACELLE POSITION
        col_valueL = ((ctrlL[COL] + 1.0f) / 2.0f * 0.75f + 0.25f - nacelleL * 0.25f) * 2.0f - 1.0f;

    tempL = computeValueFromCurve(col_valueL, -1.0f, 1.0f, leftColCurve, NUM_COL_POINTS) * colVolume;
    outputs[LEFT_S0] += tempL;
    outputs[LEFT_S1] += tempL;

    tempL = computeValueFromCurve(col_valueL, -1.0f, 1.0f, rightColCurve, NUM_COL_POINTS) * colVolume;
    outputs[RIGHT_S0] += tempL;
    outputs[RIGHT_S1] += tempL;

    //  COMPUTE NACELLE POSITION
    outputs[LEFT_SN] = ctrlL[NACELLE];
    outputs[RIGHT_SN] = ctrlL[NACELLE];

    //  CONSTRAIN THE OUTPUTS
    outputs[LEFT_S0] = constrain(outputs[LEFT_S0], -1.0f, 1.0f);
    outputs[LEFT_S1] = constrain(outputs[LEFT_S1], -1.0f, 1.0f);
    outputs[LEFT_ST] = constrain(outputs[LEFT_ST], -1.0f, 1.0f);
    outputs[LEFT_SN] = constrain(outputs[LEFT_SN], -1.0f, 1.0f);

    outputs[RIGHT_S0] = -constrain(outputs[RIGHT_S0], -1.0f, 1.0f);
    outputs[RIGHT_S1] = -constrain(outputs[RIGHT_S1], -1.0f, 1.0f);
    outputs[RIGHT_ST] =  constrain(outputs[RIGHT_ST], -1.0f, 1.0f);
    outputs[RIGHT_SN] = -constrain(outputs[RIGHT_SN], -1.0f, 1.0f);

    //  SCALE SERVOS TO PROPER RANGE
    for (iL=0; iL<NUM_MAIN_SERVOS; iL++)
    {
        outputs[iL] = servos[iL].scaleToLimits(outputs[iL]);
    }

    return NUM_MAIN_SERVOS;
}

void
V22OspreyMixer::groups_required(uint32_t &groups)
{
	/* XXX for now, hardcoded to indexes 0-3 in control group zero */
	groups |= (1 << 0);
}

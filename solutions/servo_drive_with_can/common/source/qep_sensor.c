//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//! \file   solutions\servo_drive_with_can\common\source\qep_sensor.c
//! \brief  Calculate the rotor angle and speed based on the QEP signals
//!


// **************************************************************************
// the includes

#include "qep_sensor.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals
extern QEP_SENSOR_Obj qepsensorin;

// **************************************************************************
// the functions

QEP_SENSOR_Handle QEP_SENSOR_init(void *pMemory,const size_t numBytes)
{
    QEP_SENSOR_Handle handle;
    QEP_SENSOR_Obj *obj;

    if(numBytes < sizeof(QEP_SENSOR_Obj))
    {
      return((QEP_SENSOR_Handle)NULL);
    }


    // assign the handle
    handle = (QEP_SENSOR_Handle)pMemory;

    // assign the object
    obj = (QEP_SENSOR_Obj *)handle;

    //assign EQEP base
    obj->qepHandle = EQEP1_BASE;

    return(handle);
} // end of QEP_SENSOR_init() function


void QEP_SENSOR_setParams(QEP_SENSOR_Handle qepsensorHandle)
{

    QEP_SENSOR_Obj *qepsensor = (QEP_SENSOR_Obj *)qepsensorHandle;

    qepsensor->qepsensor.PolePairs = USER_MOTOR_NUM_POLE_PAIRS;
    qepsensor->SpeedKrpm = 0.0f;
    qepsensor->SpeedK1_filter = ((float32_t)(1000.0f)/((300.0f) * USER_CTRL_PERIOD_usec)); //TODO1: Update multiply
    qepsensor->SpeedK2_filter = 0.997f;
    qepsensor->SpeedK3_filter = 1.0f - qepsensor->SpeedK2_filter;

    qepsensor->SpeedGain = ((60000.0f / (float32_t)(USER_CTRL_PERIOD_usec * USER_MOTOR_NUM_POLE_PAIRS)) / MATH_TWO_PI);

    qepsensor->qepsensor.MechScaler =
            (1.0f / (4.0f * (float32_t)USER_MOTOR_NUM_ENC_SLOTS /
                    (float32_t)(qepsensor->qepsensor.PolePairs)));

    qepsensor->lsw = ENC_WAIT_FOR_INDEX;

    qepsensor->qepsensor.CalibratedAngle = 3300;

    return;
}



void QEP_SENSOR_run(QEP_SENSOR_Handle sensorhandle)
{
    QEP_SENSOR_Obj *obj = (QEP_SENSOR_Obj *)sensorhandle;


    uint32_t qepBase = (uint32_t)sensorhandle->qepHandle;

    if(sensorhandle->lsw == ENC_CALIBRATION_DONE)
    {
        // QEP UTO and ADCSOC are carefully pre aligned
        // QEP POSLAT = QEP POSCNT at QEP UTO event that is in sync with ADCSOC
        // Therefore, read mech angle from QEP POSLAT
        sensorhandle->qepsensor.RawTheta = EQEP_getPosition(qepBase); // raw theta + offset

        sensorhandle->qepsensor.MechTheta = (sensorhandle->qepsensor.MechScaler *
                (float32_t)sensorhandle->qepsensor.RawTheta);

        //normalize -pi to pi
        sensorhandle->qepsensor.ElecTheta = fmodf(sensorhandle->qepsensor.MechTheta, (1.0f));

        sensorhandle->qepsensor.ElecTheta = ((sensorhandle->qepsensor.ElecTheta * 2.0f) - 1.0f) * MATH_PI;

        sensorhandle->Angle = sensorhandle->qepsensor.ElecTheta;

    }
    else if(sensorhandle->lsw == ENC_WAIT_FOR_INDEX)
    {
        // If QEP index pulse is found :-
        // POSILAT captures POSCNT at the first INDEX pulse.
        // Load POSINIT with POSILAT, so that at every future INDEX event,
        // POSCNT is loaded (reset) with POSINIT
        if (EQEP_getInterruptStatus(qepBase) & EQEP_INT_INDEX_EVNT_LATCH)
        {
            //QPOSINIT = QPOSILAT (QPOSILAT updates on Index edge(IEL))
            EQEP_setInitialPosition(qepBase,
                                    EQEP_getIndexPositionLatch(qepBase));

            //make QPOSCNT=QPOSINIT on Index edge
            EQEP_resetPoscnt(qepBase, EQEP_POSCNT_INIT_RISING_EDGE);


            sensorhandle->qepsensor.CalibratedAngle = EQEP_getIndexPositionLatch(qepBase);

            sensorhandle->lsw = ENC_CALIBRATION_DONE;
        }   // Keep the latched pos. at the first index

        sensorhandle->Angle = sensorhandle->CaliAngle; // forced angle input needed for calibration
    }
    else
    {
        //
        // Alignment Routine - Reset POSCNT to ZERO
        // during alignment, reset the current shaft position to zero
        //
        EQEP_setPosition(qepBase, 0);
        EQEP_clearInterruptStatus(qepBase, EQEP_INT_INDEX_EVNT_LATCH);
        EQEP_resetPoscnt(qepBase, EQEP_POSCNT_INIT_NOTHING);

        sensorhandle->Angle = 0.0f;
    }

    float32_t angleDiff = sensorhandle->Angle - sensorhandle->OldAngle;


    //The angle transion works as long as the motor does not run faster than 180 Degree per PWM cycle
    if(angleDiff < -MATH_PI)
    {
        angleDiff +=MATH_TWO_PI;
    }

    if(angleDiff > MATH_PI)
    {
        angleDiff -=MATH_TWO_PI;
    }

    angleDiff =  (sensorhandle->SpeedK2_filter * sensorhandle->OldAngleDiff) +
            (sensorhandle->SpeedK3_filter * angleDiff);

    sensorhandle->OldAngle = sensorhandle->Angle;
    sensorhandle->OldAngleDiff = angleDiff;

    sensorhandle->SpeedKrpm = angleDiff* obj->SpeedGain;		// Magnitude

    sensorhandle->SpeedHz =  sensorhandle->SpeedKrpm *
            ((float32_t)USER_MOTOR_NUM_POLE_PAIRS * 1000.0f / 60.0f);

    return;
}

// end of file

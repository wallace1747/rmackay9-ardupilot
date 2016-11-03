#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of range beacon measurements
void NavEKF2_core::SelectRngBcnFusion()
{
    // read range data from the sensor and check for new data in the buffer
    readRngBcnData();

    // Determine if we need to fuse range beacon data on this time step
    if (rngBcnDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        FuseRngBcn();
    }
}

void NavEKF2_core::FuseRngBcn()
{
    // declarations
    float pn;
    float pe;
    float pd;
    float bcn_pn;
    float bcn_pe;
    float bcn_pd;
    const float R_BCN = sq(MAX(rngBcnDataDelayed.rngErr , 0.1f));
    float rngPred;

    // health is set bad until test passed
    rngBcnHealth = false;

    // copy required states to local variable names
    pn = stateStruct.position.x;
    pe = stateStruct.position.y;
    pd = stateStruct.position.z;
    bcn_pn = rngBcnDataDelayed.beacon_posNED.x;
    bcn_pe = rngBcnDataDelayed.beacon_posNED.y;
    bcn_pd = rngBcnDataDelayed.beacon_posNED.z;

    // predicted range
    Vector3f deltaPosNED = stateStruct.position - rngBcnDataDelayed.beacon_posNED;
    rngPred = deltaPosNED.length();

    // perform fusion of range measurement
    if (rngPred > 0.1f)
    {
        // calculate observation jacobians
        float H_BCN[24] = {};
        float t2 = bcn_pd-pd;
        float t3 = bcn_pe-pe;
        float t4 = bcn_pn-pn;
        float t5 = t2*t2;
        float t6 = t3*t3;
        float t7 = t4*t4;
        float t8 = t5+t6+t7;
        float t9 = 1.0f/sqrt(t8);
        H_BCN[6] = -t4*t9;
        H_BCN[7] = -t3*t9;
        H_BCN[8] = -t2*t9;

        // calculate Kalman gains
        float t10 = P[8][8]*H_BCN[8];
        float t11 = P[7][8]*H_BCN[7];
        float t12 = P[6][8]*H_BCN[6];
        float t13 = t10+t11+t12;
        float t14 = t2*t9*t13;
        float t15 = P[8][7]*H_BCN[8];
        float t16 = P[7][7]*H_BCN[7];
        float t17 = P[6][7]*H_BCN[6];
        float t18 = t15+t16+t17;
        float t19 = t3*t9*t18;
        float t20 = P[8][6]*H_BCN[8];
        float t21 = P[7][6]*H_BCN[7];
        float t22 = P[6][6]*H_BCN[6];
        float t23 = t20+t21+t22;
        float t24 = t4*t9*t23;
        varInnovRngBcn = R_BCN+t14+t19+t24;
        float t26;
        if (varInnovRngBcn >= R_BCN) {
            t26 = 1.0/varInnovRngBcn;
            faultStatus.bad_rngbcn = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_rngbcn = true;
            return;
        }
        for (unsigned j = 0; j<=15; j++) {
            Kfusion[j] = t26*(P[j][6]*H_BCN[6]+P[j][7]*H_BCN[7]+P[j][8]*H_BCN[8]);
        }
        if (!inhibitMagStates) {
            for (unsigned j = 16; j<=21; j++) {
                Kfusion[j] = t26*(P[j][6]*H_BCN[6]+P[j][7]*H_BCN[7]+P[j][8]*H_BCN[8]);
            }
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }
        for (unsigned j = 22; j<=23; j++) {
            Kfusion[j] = t26*(P[j][6]*H_BCN[6]+P[j][7]*H_BCN[7]+P[j][8]*H_BCN[8]);
        }

        // calculate measurement innovation
        innovRngBcn = rngPred - rngBcnDataDelayed.rng;

        // calculate the innovation consistency test ratio
        rngBcnTestRatio = sq(innovRngBcn) / (sq(MAX(0.01f * (float)frontend->_rngBcnInnovGate, 1.0f)) * varInnovRngBcn);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        rngBcnHealth = ((rngBcnTestRatio < 1.0f) || badIMUdata);

        // test the ratio before fusing data
        if (rngBcnHealth) {

            // restart the counter
            lastRngBcnPassTime_ms = imuSampleTime_ms;

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=5; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 6; j<=8; j++) {
                    KH[i][j] = Kfusion[i] * H_BCN[j];
                }
                for (unsigned j = 9; j<=23; j++) {
                    KH[i][j] = 0.0f;
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][6] * P[6][j];
                    res += KH[i][7] * P[7][j];
                    res += KH[i][8] * P[8][j];
                    KHP[i][j] = res;
                }
            }
            // Check that we are not going to drive any variances negative and skip the update if so
            bool healthyFusion = true;
            for (uint8_t i= 0; i<=stateIndexLim; i++) {
                if (KHP[i][i] > P[i][i]) {
                    healthyFusion = false;
                }
            }
            if (healthyFusion) {
                // update the covariance matrix
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }

                // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
                ForceSymmetry();
                ConstrainVariances();

                // update the states
                // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
                stateStruct.angErr.zero();

                // correct the state vector
                for (uint8_t j= 0; j<=stateIndexLim; j++) {
                    statesArray[j] = statesArray[j] - Kfusion[j] * innovRngBcn;
                }

                // the first 3 states represent the angular misalignment vector. This is
                // is used to correct the estimated quaternion on the current time step
                stateStruct.quat.rotate(stateStruct.angErr);

                // record healthy fusion
                faultStatus.bad_rngbcn = false;

            } else {
                // record bad fusion
                faultStatus.bad_rngbcn = true;

            }
        }
    }
}

#endif // HAL_CPU_CLASS

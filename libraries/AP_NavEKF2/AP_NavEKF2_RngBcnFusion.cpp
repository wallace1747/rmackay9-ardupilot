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
        float SH_BCN = 1.0f /rngPred;
        float H_BCN[24] = {};
        H_BCN[6] = -SH_BCN*(bcn_pn - pn);
        H_BCN[7] = -SH_BCN*(bcn_pe - pe);
        H_BCN[8] = -SH_BCN*(bcn_pd - pd);

        // calculate Kalman gains
        float SK_BCN[4];
        varInnovRngBcn = (R_BCN + SH_BCN*(bcn_pd - pd)*(P[8][8]*SH_BCN*(bcn_pd - pd) + P[7][8]*SH_BCN*(bcn_pe - pe) + P[6][8]*SH_BCN*(bcn_pn - pn)) + SH_BCN*(bcn_pe - pe)*(P[8][7]*SH_BCN*(bcn_pd - pd) + P[7][7]*SH_BCN*(bcn_pe - pe) + P[6][7]*SH_BCN*(bcn_pn - pn)) + SH_BCN*(bcn_pn - pn)*(P[8][6]*SH_BCN*(bcn_pd - pd) + P[7][6]*SH_BCN*(bcn_pe - pe) + P[6][6]*SH_BCN*(bcn_pn - pn)));
        if (varInnovRngBcn >= R_BCN) {
            SK_BCN[0] = 1.0f / varInnovRngBcn;
            SK_BCN[1] = bcn_pn - pn;
            SK_BCN[2] = bcn_pe - pe;
            SK_BCN[3] = bcn_pd - pd;
            faultStatus.bad_rngbcn = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_rngbcn = true;
            return;
        }

        Kfusion[0] = -SK_BCN[0]*(P[0][6]*SK_BCN[1]*SK_BCN[0] + P[0][7]*SK_BCN[2]*SK_BCN[0] + P[0][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[1] = -SK_BCN[0]*(P[1][6]*SK_BCN[1]*SK_BCN[0] + P[1][7]*SK_BCN[2]*SK_BCN[0] + P[1][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[2] = -SK_BCN[0]*(P[2][6]*SK_BCN[1]*SK_BCN[0] + P[2][7]*SK_BCN[2]*SK_BCN[0] + P[2][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[3] = -SK_BCN[0]*(P[3][6]*SK_BCN[1]*SK_BCN[0] + P[3][7]*SK_BCN[2]*SK_BCN[0] + P[3][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[4] = -SK_BCN[0]*(P[4][6]*SK_BCN[1]*SK_BCN[0] + P[4][7]*SK_BCN[2]*SK_BCN[0] + P[4][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[5] = -SK_BCN[0]*(P[5][6]*SK_BCN[1]*SK_BCN[0] + P[5][7]*SK_BCN[2]*SK_BCN[0] + P[5][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[6] = -SK_BCN[0]*(P[6][6]*SK_BCN[1]*SK_BCN[0] + P[6][7]*SK_BCN[2]*SK_BCN[0] + P[6][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[7] = -SK_BCN[0]*(P[7][6]*SK_BCN[1]*SK_BCN[0] + P[7][7]*SK_BCN[2]*SK_BCN[0] + P[7][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[8] = -SK_BCN[0]*(P[8][6]*SK_BCN[1]*SK_BCN[0] + P[8][7]*SK_BCN[2]*SK_BCN[0] + P[8][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[9] = -SK_BCN[0]*(P[9][6]*SK_BCN[1]*SK_BCN[0] + P[9][7]*SK_BCN[2]*SK_BCN[0] + P[9][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[10] = -SK_BCN[0]*(P[10][6]*SK_BCN[1]*SK_BCN[0] + P[10][7]*SK_BCN[2]*SK_BCN[0] + P[10][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[11] = -SK_BCN[0]*(P[11][6]*SK_BCN[1]*SK_BCN[0] + P[11][7]*SK_BCN[2]*SK_BCN[0] + P[11][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[12] = -SK_BCN[0]*(P[12][6]*SK_BCN[1]*SK_BCN[0] + P[12][7]*SK_BCN[2]*SK_BCN[0] + P[12][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[13] = -SK_BCN[0]*(P[13][6]*SK_BCN[1]*SK_BCN[0] + P[13][7]*SK_BCN[2]*SK_BCN[0] + P[13][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[14] = -SK_BCN[0]*(P[14][6]*SK_BCN[1]*SK_BCN[0] + P[14][7]*SK_BCN[2]*SK_BCN[0] + P[14][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[15] = -SK_BCN[0]*(P[15][6]*SK_BCN[1]*SK_BCN[0] + P[15][7]*SK_BCN[2]*SK_BCN[0] + P[15][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[22] = -SK_BCN[0]*(P[22][6]*SK_BCN[1]*SK_BCN[0] + P[22][7]*SK_BCN[2]*SK_BCN[0] + P[22][8]*SK_BCN[3]*SK_BCN[0]);
        Kfusion[23] = -SK_BCN[0]*(P[23][6]*SK_BCN[1]*SK_BCN[0] + P[23][7]*SK_BCN[2]*SK_BCN[0] + P[23][8]*SK_BCN[3]*SK_BCN[0]);
        if (!inhibitMagStates) {
            Kfusion[16] = -SK_BCN[0]*(P[16][6]*SK_BCN[1]*SK_BCN[0] + P[16][7]*SK_BCN[2]*SK_BCN[0] + P[16][8]*SK_BCN[3]*SK_BCN[0]);
            Kfusion[17] = -SK_BCN[0]*(P[17][6]*SK_BCN[1]*SK_BCN[0] + P[17][7]*SK_BCN[2]*SK_BCN[0] + P[17][8]*SK_BCN[3]*SK_BCN[0]);
            Kfusion[18] = -SK_BCN[0]*(P[18][6]*SK_BCN[1]*SK_BCN[0] + P[18][7]*SK_BCN[2]*SK_BCN[0] + P[18][8]*SK_BCN[3]*SK_BCN[0]);
            Kfusion[19] = -SK_BCN[0]*(P[19][6]*SK_BCN[1]*SK_BCN[0] + P[19][7]*SK_BCN[2]*SK_BCN[0] + P[19][8]*SK_BCN[3]*SK_BCN[0]);
            Kfusion[20] = -SK_BCN[0]*(P[20][6]*SK_BCN[1]*SK_BCN[0] + P[20][7]*SK_BCN[2]*SK_BCN[0] + P[20][8]*SK_BCN[3]*SK_BCN[0]);
            Kfusion[21] = -SK_BCN[0]*(P[21][6]*SK_BCN[1]*SK_BCN[0] + P[21][7]*SK_BCN[2]*SK_BCN[0] + P[21][8]*SK_BCN[3]*SK_BCN[0]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
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

    // force the covariance matrix to me symmetrical and limit the variances to prevent ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();
}

#endif // HAL_CPU_CLASS

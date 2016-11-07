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
    if (rngBcnDataToFuse) {
        if (PV_AidingMode == AID_ABSOLUTE) {
            // Normal operating mode is to fuse the range data into the main filter
            FuseRngBcn();
        } else {
            // If we aren't able to use the data in the main filter, use a simple 3-state filter to estimte position only
            FuseRngBcnStatic();
        }
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
        float t10 = P[8][8]*t2*t9;
        float t11 = P[7][8]*t3*t9;
        float t12 = P[6][8]*t4*t9;
        float t13 = t10+t11+t12;
        float t14 = t2*t9*t13;
        float t15 = P[8][7]*t2*t9;
        float t16 = P[7][7]*t3*t9;
        float t17 = P[6][7]*t4*t9;
        float t18 = t15+t16+t17;
        float t19 = t3*t9*t18;
        float t20 = P[8][6]*t2*t9;
        float t21 = P[7][6]*t3*t9;
        float t22 = P[6][6]*t4*t9;
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

        Kfusion[0] = -t26*(P[0][6]*t4*t9+P[0][7]*t3*t9+P[0][8]*t2*t9);
        Kfusion[1] = -t26*(P[1][6]*t4*t9+P[1][7]*t3*t9+P[1][8]*t2*t9);
        Kfusion[2] = -t26*(P[2][6]*t4*t9+P[2][7]*t3*t9+P[2][8]*t2*t9);
        Kfusion[3] = -t26*(P[3][6]*t4*t9+P[3][7]*t3*t9+P[3][8]*t2*t9);
        Kfusion[4] = -t26*(P[4][6]*t4*t9+P[4][7]*t3*t9+P[4][8]*t2*t9);
        Kfusion[5] = -t26*(P[5][6]*t4*t9+P[5][7]*t3*t9+P[5][8]*t2*t9);
        Kfusion[6] = -t26*(t22+P[6][7]*t3*t9+P[6][8]*t2*t9);
        Kfusion[7] = -t26*(t16+P[7][6]*t4*t9+P[7][8]*t2*t9);
        Kfusion[8] = -t26*(t10+P[8][6]*t4*t9+P[8][7]*t3*t9);
        Kfusion[9] = -t26*(P[9][6]*t4*t9+P[9][7]*t3*t9+P[9][8]*t2*t9);
        Kfusion[10] = -t26*(P[10][6]*t4*t9+P[10][7]*t3*t9+P[10][8]*t2*t9);
        Kfusion[11] = -t26*(P[11][6]*t4*t9+P[11][7]*t3*t9+P[11][8]*t2*t9);
        Kfusion[12] = -t26*(P[12][6]*t4*t9+P[12][7]*t3*t9+P[12][8]*t2*t9);
        Kfusion[13] = -t26*(P[13][6]*t4*t9+P[13][7]*t3*t9+P[13][8]*t2*t9);
        Kfusion[14] = -t26*(P[14][6]*t4*t9+P[14][7]*t3*t9+P[14][8]*t2*t9);
        Kfusion[15] = -t26*(P[15][6]*t4*t9+P[15][7]*t3*t9+P[15][8]*t2*t9);
        if (!inhibitMagStates) {
            Kfusion[16] = -t26*(P[16][6]*t4*t9+P[16][7]*t3*t9+P[16][8]*t2*t9);
            Kfusion[17] = -t26*(P[17][6]*t4*t9+P[17][7]*t3*t9+P[17][8]*t2*t9);
            Kfusion[18] = -t26*(P[18][6]*t4*t9+P[18][7]*t3*t9+P[18][8]*t2*t9);
            Kfusion[19] = -t26*(P[19][6]*t4*t9+P[19][7]*t3*t9+P[19][8]*t2*t9);
            Kfusion[20] = -t26*(P[20][6]*t4*t9+P[20][7]*t3*t9+P[20][8]*t2*t9);
            Kfusion[21] = -t26*(P[21][6]*t4*t9+P[21][7]*t3*t9+P[21][8]*t2*t9);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }
        Kfusion[22] = -t26*(P[22][6]*t4*t9+P[22][7]*t3*t9+P[22][8]*t2*t9);
        Kfusion[23] = -t26*(P[23][6]*t4*t9+P[23][7]*t3*t9+P[23][8]*t2*t9);

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

/*
Use range beaon measurements to calculate a static position using a 3-state EKF algorithm.
Algorihtm based on the following:
https://github.com/priseborough/InertialNav/blob/master/derivations/range_beacon.m
*/
void NavEKF2_core::FuseRngBcnStatic()
{
    /*
    The first thing to do is to check if we have started the alignment and if not, initialise the
    states and covariance to a first guess. To do this iterate through the avilable beacons and then
    initialise the initial position to the mean beacon position. The initial position uncertainty
    is set to the mean range measurement.
    */
    if (!rngBcnAlignmentStarted) {
        if (rngBcnDataDelayed.beacon_ID != lastBeaconIndex) {
            rngBcnPosSum += rngBcnDataDelayed.beacon_posNED;
            lastBeaconIndex = rngBcnDataDelayed.beacon_ID;
            rngSum += rngBcnDataDelayed.rng;
            numBcnMeas++;
        }
        if (numBcnMeas >= 100) {
            rngBcnAlignmentStarted = true;
            float tempVar = 1.0f / (float)numBcnMeas;
            receiverPos = rngBcnPosSum * tempVar;
            receiverPosCov[2][2] = receiverPosCov[1][1] = receiverPosCov[0][0] = rngSum * tempVar;
            lastBeaconIndex  = 0;
            numBcnMeas = 0;
            rngBcnPosSum.zero();
            rngSum = 0.0f;
        }
    }

    if (rngBcnAlignmentStarted && !rngBcnAlignmentCompleted) {
        numBcnMeas++;

        // Add some process noise to the states at each time step
        for (uint8_t i= 0; i<=2; i++) {
            receiverPosCov[i][i] += 0.1f;
        }

        // get the estimated range measurement variance
        const float R_RNG = sq(MAX(rngBcnDataDelayed.rngErr , 0.1f));

        // calculate the observation jacobian
        float t2 = rngBcnDataDelayed.beacon_posNED.z - receiverPos.z;
        float t3 = rngBcnDataDelayed.beacon_posNED.y - receiverPos.y;
        float t4 = rngBcnDataDelayed.beacon_posNED.x - receiverPos.x;
        float t5 = t2*t2;
        float t6 = t3*t3;
        float t7 = t4*t4;
        float t8 = t5+t6+t7;
        if (t8 < 0.1f) {
            // calculation will be badly conditioned
            return;
        }
        float t9 = 1.0f/sqrt(t8);
        float t10 = rngBcnDataDelayed.beacon_posNED.x*2.0f;
        float t15 = receiverPos.x*2.0f;
        float t11 = t10-t15;
        float t12 = rngBcnDataDelayed.beacon_posNED.y*2.0f;
        float t14 = receiverPos.y*2.0f;
        float t13 = t12-t14;
        float t16 = rngBcnDataDelayed.beacon_posNED.z*2.0f;
        float t18 = receiverPos.z*2.0f;
        float t17 = t16-t18;
        float H_RNG[3];
        H_RNG[0] = -t9*t11*0.5f;
        H_RNG[1] = -t9*t13*0.5f;
        H_RNG[2] = -t9*t17*0.5f;

        // calculate the Kalman gains
        float t19 = receiverPosCov[0][0]*t9*t11*0.5f;
        float t20 = receiverPosCov[1][1]*t9*t13*0.5f;
        float t21 = receiverPosCov[0][1]*t9*t11*0.5f;
        float t22 = receiverPosCov[2][1]*t9*t17*0.5f;
        float t23 = t20+t21+t22;
        float t24 = t9*t13*t23*0.5f;
        float t25 = receiverPosCov[1][2]*t9*t13*0.5f;
        float t26 = receiverPosCov[0][2]*t9*t11*0.5f;
        float t27 = receiverPosCov[2][2]*t9*t17*0.5f;
        float t28 = t25+t26+t27;
        float t29 = t9*t17*t28*0.5f;
        float t30 = receiverPosCov[1][0]*t9*t13*0.5f;
        float t31 = receiverPosCov[2][0]*t9*t17*0.5f;
        float t32 = t19+t30+t31;
        float t33 = t9*t11*t32*0.5f;
        varInnovRngBcn = R_RNG+t24+t29+t33;
        float t35 = 1.0f/varInnovRngBcn;
        float K_RNG[3];
        K_RNG[0] = -t35*(t19+receiverPosCov[0][1]*t9*t13*0.5f+receiverPosCov[0][2]*t9*t17*0.5f);
        K_RNG[1] = -t35*(t20+receiverPosCov[1][0]*t9*t11*0.5f+receiverPosCov[1][2]*t9*t17*0.5f);
        K_RNG[2] = -t35*(t27+receiverPosCov[2][0]*t9*t11*0.5f+receiverPosCov[2][1]*t9*t13*0.5f);

        // calculate range measurement innovation
        Vector3f deltaPosNED = receiverPos - rngBcnDataDelayed.beacon_posNED;
        innovRngBcn = deltaPosNED.length() - rngBcnDataDelayed.rng;

        // update the state
        receiverPos.x -= K_RNG[0] * innovRngBcn;
        receiverPos.y -= K_RNG[1] * innovRngBcn;
        receiverPos.z -= K_RNG[2] * innovRngBcn;

        // calculate the covariance correction
        for (unsigned i = 0; i<=2; i++) {
            for (unsigned j = 0; j<=2; j++) {
                KH[i][j] = K_RNG[i] * H_RNG[j];
            }
        }
        for (unsigned j = 0; j<=2; j++) {
            for (unsigned i = 0; i<=2; i++) {
                ftype res = 0;
                res += KH[i][0] * receiverPosCov[0][j];
                res += KH[i][1] * receiverPosCov[1][j];
                res += KH[i][2] * receiverPosCov[2][j];
                KHP[i][j] = res;
            }
        }
        // prevent negative variances
        for (uint8_t i= 0; i<=2; i++) {
            if (receiverPosCov[i][i] < 0.0f) {
                receiverPosCov[i][i] = 0.0f;
                KHP[i][i] = 0.0f;
            } else if (KHP[i][i] > receiverPosCov[i][i]) {
                KHP[i][i] = receiverPosCov[i][i];
            }
        }
        // apply the covariance correction
        for (uint8_t i= 0; i<=2; i++) {
            for (uint8_t j= 0; j<=2; j++) {
                receiverPosCov[i][j] -= KHP[i][j];
            }
        }
        // ensure the covariance matrix is symmetric
        for (uint8_t i=1; i<=2; i++) {
            for (uint8_t j=0; j<=i-1; j++) {
                float temp = 0.5f*(receiverPosCov[i][j] + receiverPosCov[j][i]);
                receiverPosCov[i][j] = temp;
                receiverPosCov[j][i] = temp;
            }
        }

        if (numBcnMeas >= 100) {
            rngBcnAlignmentCompleted = true;
        }
    }
}
#endif // HAL_CPU_CLASS

/* Sinh tu dong boi tools/fit_kn.py - khong sua tay.
 * d[mm] = IR_K[i] * value^(-IR_N[i]);  he so = 0 -> sensor chua hieu chuan */
#ifndef SENSOR_CALIB_H
#define SENSOR_CALIB_H

#define IR_CALIB_VALID 1

static const float IR_K[5] = {5209.34f, 4859.09f, 8031.35f, 4444.01f, 7330.16f};
static const float IR_N[5] = {0.65724f, 0.674958f, 0.653585f, 0.672007f, 0.631336f};

#endif

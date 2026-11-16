#pragma once
#include <cstdint>

namespace mm::cfg {

constexpr int   N         = 16;
constexpr float CELL      = 180.0f;
constexpr float HALF      = 90.0f;
constexpr float DIAG_STEP = 127.2792206f;
constexpr float WALL_HALF = 6.0f;
constexpr float CORRIDOR  = 168.0f;
constexpr int   GOAL_X0 = 7, GOAL_Y0 = 7;

constexpr float DT_S = 0.001f;

constexpr float TRACK_MM          = 84.3f;
constexpr float COUNTS_PER_MM_L   = 3.038f;
constexpr float COUNTS_PER_MM_R   = 3.038f;
constexpr int   ENC_SIGN_L        = +1;
constexpr int   ENC_SIGN_R        = -1;
constexpr int   MOTOR_SIGN_L      = +1;
constexpr int   MOTOR_SIGN_R      = +1;
constexpr float GYRO_SIGN         = -1.0f;
constexpr float GYRO_SCALE        = 1.0029f;

constexpr float ROBOT_HALF_WIDTH  = 35.0f;
constexpr float ROBOT_FRONT       = 65.0f;
constexpr float ROBOT_BACK        = 25.0f;
constexpr float START_OFFSET      = -(HALF - WALL_HALF - ROBOT_BACK - 2.0f);

constexpr float VBAT_NOMINAL = 7.6f;
constexpr float KV_MOTOR     = 0.00244f;
constexpr float KA_MOTOR     = 0.000358f;
constexpr float KS_MOTOR     = 0.517f;
constexpr float ROT_INERTIA  = 0.74f;
constexpr float DUTY_LIMIT   = 0.95f;
constexpr float V_HEADROOM   = 0.88f;
constexpr float V_WHEEL_MAX  = (V_HEADROOM * VBAT_NOMINAL - KS_MOTOR) / KV_MOTOR;

constexpr float WN_POS = 14.0f;
constexpr float WN_ANG = 30.0f;
constexpr float ZETA   = 0.8f;
constexpr float G_ROT  = 2.0f / TRACK_MM;
constexpr float KP_POS = KA_MOTOR * WN_POS * WN_POS;
constexpr float KD_POS = (2 * ZETA * WN_POS * KA_MOTOR - KV_MOTOR) > 0 ? (2 * ZETA * WN_POS * KA_MOTOR - KV_MOTOR) : 0.0f;
constexpr float KI_POS = KP_POS * 2.0f;
constexpr float KP_ANG = KA_MOTOR * ROT_INERTIA * WN_ANG * WN_ANG / G_ROT;
constexpr float KD_ANG = (2 * ZETA * WN_ANG * KA_MOTOR * ROT_INERTIA - KV_MOTOR) > 0
                             ? (2 * ZETA * WN_ANG * KA_MOTOR * ROT_INERTIA - KV_MOTOR) / G_ROT : 0.0f;
constexpr float KI_ANG = KP_ANG * 4.0f;
constexpr float I_LIMIT_V = 1.5f;
constexpr float WALL_LAT_BW = 4.0f;
constexpr float KWALL_MAX   = 0.015f;
constexpr float WALL_ANG_LIMIT = 0.20f;
#ifndef MM_DIAG_POST_CTRL
#define MM_DIAG_POST_CTRL 0
#endif
constexpr bool  DIAG_POST_CTRL = MM_DIAG_POST_CTRL;
constexpr float DIAG_POST_NOM  = 63.64f - WALL_HALF * 1.41421f - 26.0f;
constexpr float DIAG_POST_SEEN = 2.2f * DIAG_POST_NOM;
constexpr float KWALL_DIAG     = 0.004f;
constexpr float WALL_HEADING_GAIN = 3.0f;
constexpr float KP_FRONT_DIST = 0.08f;
constexpr float KP_FRONT_ANG  = 25.0f;

enum IrId : uint8_t { IR_F = 0, IR_DL, IR_DR, IR_L, IR_R, IR_COUNT };
struct IrPose { float fwd, lat, ang; };
constexpr IrPose IR_POSE[IR_COUNT] = {
  {58.0f,   0.0f,  0.0f},
  {50.0f,  22.0f,  0.7853982f},
  {50.0f, -22.0f, -0.7853982f},
  {36.0f,  37.8f,  1.5707963f},
  {36.0f, -38.5f, -1.5707963f},
};
constexpr float IR_SAT_MM[IR_COUNT] = {35.0f, 30.0f, 30.0f, 41.0f, 31.0f};
constexpr float IR_NONE          = 999.0f;
constexpr float SIDE_NOM_L       = CORRIDOR / 2 - 37.8f;
constexpr float SIDE_NOM_R       = CORRIDOR / 2 - 38.5f;
constexpr float SIDE_NOMINAL     = 0.5f * (SIDE_NOM_L + SIDE_NOM_R);
constexpr float SIDE_WALL_TH     = 115.0f;
constexpr float SIDE_FILT_MM     = 10.0f;
constexpr float FRONT_ALIGN_X    = CORRIDOR / 2;
constexpr float DIAG_FRONT_NOM   = (FRONT_ALIGN_X - 50.0f) * 1.41421356f;
constexpr float SIDE_EDGE_HYST   = 12.0f;
constexpr float FRONT_WALL_TH    = 160.0f;
constexpr float FRONT_MAYBE_TH   = 105.0f;
constexpr int   FRONT_MAYBE_PCT  = 75;
constexpr float LOOK_FRONT_TH    = 95.0f;
constexpr float LOOK_AMBIG_MM    = 130.0f;
constexpr float LOOK_RETRY_MM    = 30.0f;
constexpr float LOOK_STOP_MM     = 30.0f;
constexpr float CREEP_V          = 100.0f;
constexpr float CREEP_TARGET_MM  = 38.0f;
constexpr uint32_t LOOK_WAIT_MS  = 30;
constexpr uint32_t LOOK_MS       = 60;
constexpr bool  FOLLOW_CURVE     = false;
constexpr bool  EXPLORE_LOOK     = true;
constexpr float GUARD_S0         = FOLLOW_CURVE ? 5.0f : 15.0f;
constexpr float GUARD_S1         = 75.0f;
constexpr float GUARD_MM         = 95.0f;
#ifndef MM_FRONT_MOVE_BIAS
#define MM_FRONT_MOVE_BIAS 25.0f
#endif
constexpr float FRONT_MOVE_BIAS  = MM_FRONT_MOVE_BIAS;
constexpr float DYN_FWD_MIN      = 55.0f;
constexpr float DYN_FWD_MAX      = 110.0f;
constexpr float DYN_EDGE_STOP_MM = 70.0f;
constexpr float DYN_WAIT_MARGIN  = 10.0f;
constexpr float FRONT_STOP_MM    = CORRIDOR / 2 - 58.0f;
constexpr float SIDE_SENSOR_FWD  = 36.0f;
constexpr bool  EDGE_CORR        = true;
constexpr float SENSE_BEFORE_EDGE = 8.0f;
constexpr float SENSE_GAP        = 12.0f;
constexpr float WALL_END_AXLE    = SIDE_SENSOR_FWD - WALL_HALF;
constexpr float WALL_START_AXLE  = SIDE_SENSOR_FWD + WALL_HALF;
constexpr float EDGE_CORR_MAX    = 25.0f;
constexpr float SIDE_JUMP_MM     = 14.0f;

constexpr float CLEAR_MIN_MM   = 12.0f;
constexpr float FAULT_POS_MM   = 60.0f;
constexpr float FAULT_ANG_RAD  = 0.6f;
constexpr uint16_t FAULT_MS    = 150;

struct RunParams {
  float v_search;
  float v_max;
  float v_diag;
  float accel;
  float decel;
  float a_lat;
  float v_turn_cap;
  bool  diagonal;
  bool  big_turns;
};
constexpr RunParams RUN_SEARCH = {300.0f,  300.0f,  300.0f, 2000.0f, 2000.0f, 2500.0f,  300.0f, false, false};
constexpr RunParams RUN_RETURN = {300.0f,  800.0f,  600.0f, 2500.0f, 2500.0f, 3000.0f,  600.0f, false, true};
constexpr RunParams RUN_SAFE   = {300.0f, 1200.0f,  900.0f, 3500.0f, 3500.0f, 4000.0f,  900.0f, true,  true};
constexpr RunParams RUN_FAST   = {300.0f, 2000.0f, 1400.0f, 5500.0f, 5000.0f, 6000.0f, 1300.0f, true,  true};
constexpr float PIVOT_W_MAX = 8.0f;
constexpr float PIVOT_ALPHA = 60.0f;
constexpr float FRONT_ALIGN_S = 0.80f;

constexpr float EXPLORE_TIME_LIMIT_S = 240.0f;
constexpr uint16_t COST_F_FIRST = 150, COST_F_NEXT = 80, COST_TURN = 210, COST_ZIGZAG = 95,
                   COST_SAME_TURN = 330, COST_PIVOT90 = 400, COST_PIVOT180 = 700;
constexpr uint32_t RUN_REST_MS = 2600;

constexpr uint32_t BTN_STABLE_MS = 30;
constexpr uint32_t BTN_RESET_HOLD_MS = 2000;
constexpr uint32_t BTN_LONG_MS = 800;
constexpr bool LOCK_FOLLOW = false;
#ifndef MM_GOAL_CORNER
#define MM_GOAL_CORNER 0
#endif
#ifndef MM_START_CORNER
#define MM_START_CORNER 0
#endif
#ifndef MM_EDGE_RISE
#define MM_EDGE_RISE 1
#endif
#ifndef MM_FA_FILTER
#define MM_FA_FILTER 1
#endif
constexpr bool START_CORNER_ALIGN = MM_START_CORNER;
constexpr bool EDGE_RISE = MM_EDGE_RISE;
constexpr bool FA_FILTER = MM_FA_FILTER;
constexpr bool GOAL_CORNER_ALIGN = MM_GOAL_CORNER;
constexpr bool FAST_RETURN = false;

}

#pragma once
#include <cstdint>
#include "planner.h"

namespace mm {

enum class MoveType : uint8_t { STRAIGHT, DIAG, TURN, PIVOT, FRONT_ALIGN, WAIT };

enum TurnType : uint8_t {
  T_SEARCH90,
  T_FAST90,
  T_FAST180,
  T_IN45,
  T_OUT45,
  T_IN135,
  T_OUT135,
  T_V90,
  T_DYN,
  T_COUNT
};

struct TurnGeom {
  float pre, curve, post;
  float theta;
  float kmax;
  float end_fwd, end_lat;
};
extern TurnGeom g_turn[T_COUNT];
extern float g_turn_frac[T_COUNT];
void init_turns();
void set_dyn_turn(float fwd, float lat, float theta);
float turn_curvature(const TurnGeom& g, float s);

enum : uint8_t { F_SENSE = 1, F_WALL = 2, F_EDGE = 4, F_STOP_FRONT = 8,
                 F_FGUARD = 16 };

struct Move {
  MoveType type = MoveType::STRAIGHT;
  uint8_t  turn = 0;
  int8_t   dir = 0;
  uint8_t  flags = 0;
  float    dist = 0;
  float    v_max = 0;
  float    v_end = 0;
  float    accel = 1000, decel = 1000;
  float    sense_at = 0;
  float    edge_phase = -1;
};

Move mk_straight(float dist, const cfg::RunParams& rp, float v_end, uint8_t flags, float edge_phase);
Move mk_turn(TurnType t, int8_t dir, const cfg::RunParams& rp);
Move mk_pivot(float angle_rad);
Move mk_front_align();
Move mk_wait(float seconds);

float turn_speed(TurnType t, const cfg::RunParams& rp);
float straight_vmax(float v);

int compile_route(const Route& r, Dir heading, float start_offset, const cfg::RunParams& rp,
                  Move* out, int max_moves);

void plan_speeds(Move* mv, int n);

bool verify_moves(const Move* mv, int n, uint8_t start_cell, Dir heading, float start_offset,
                  uint8_t end_cell, Dir end_heading);

float path_clearance(const Move* mv, int n, uint8_t start_cell, Dir heading, float start_offset, const Maze& m);

}

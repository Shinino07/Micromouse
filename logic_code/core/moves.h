#pragma once
#include <cstdint>
#include "planner.h"

namespace mm {

enum class MoveType : uint8_t { STRAIGHT, DIAG, TURN, PIVOT, FRONT_ALIGN, WAIT };

enum TurnType : uint8_t {
  T_SEARCH90,  // mep o -> mep o (do duong)
  T_FAST90,    // tam o -> tam o
  T_FAST180,   // tam o -> tam o
  T_IN45,      // tam o -> mep o (cheo)
  T_OUT45,     // mep o (cheo) -> tam o
  T_IN135,
  T_OUT135,
  T_V90,       // cheo -> cheo
  T_COUNT
};

// Duong cua: thang pre -> cong (do cong k = kmax*sin^2) dai curve -> thang post
struct TurnGeom {
  float pre, curve, post;
  float theta;  // rad (duong)
  float kmax;   // 1/mm
  float end_fwd, end_lat;  // dich chuyen node->node trong he truc luc bat dau (lat ve phia re)
};
extern TurnGeom g_turn[T_COUNT];
extern float g_turn_frac[T_COUNT];  // ti le do dai doan cong so voi lon nhat (<1: cua gat hon, co doan thang)
void init_turns();
float turn_curvature(const TurnGeom& g, float s);  // s tinh tu dau move

enum : uint8_t { F_SENSE = 1, F_WALL = 2, F_EDGE = 4, F_STOP_FRONT = 8 };

struct Move {
  MoveType type = MoveType::STRAIGHT;
  uint8_t  turn = 0;
  int8_t   dir = 0;        // +1 trai, -1 phai
  uint8_t  flags = 0;
  float    dist = 0;       // mm (thang/cheo) | rad co dau (xoay) | s (cho)
  float    v_max = 0;
  float    v_end = 0;
  float    accel = 1000, decel = 1000;
  float    sense_at = 0;   // mm tu dau move
  float    edge_phase = -1;  // mm tu dau move toi mep o dau tien phia truoc (<0: khong sua)
};

Move mk_straight(float dist, const cfg::RunParams& rp, float v_end, uint8_t flags, float edge_phase);
Move mk_turn(TurnType t, int8_t dir, const cfg::RunParams& rp);
Move mk_pivot(float angle_rad);
Move mk_front_align();
Move mk_wait(float seconds);

float turn_speed(TurnType t, const cfg::RunParams& rp);
float straight_vmax(float v);

// Bien duong theo o thanh chuoi chuyen dong. start_offset: truc banh truoc(+)/sau(-) tam o dau.
// Duong ket thuc o tam o cuoi, van toc 0. Tra ve so move, -1 neu loi.
int compile_route(const Route& r, Dir heading, float start_offset, const cfg::RunParams& rp,
                  Move* out, int max_moves);

// Lap van toc cuoi moi move (quet nguoc) de luon phanh kip
void plan_speeds(Move* mv, int n);

// Kiem tra hinh hoc: tich phan cac move, so voi tam o cuoi va huong cuoi
bool verify_moves(const Move* mv, int n, uint8_t start_cell, Dir heading, float start_offset,
                  uint8_t end_cell, Dir end_heading);

// Khe ho nho nhat (mm) giua vien robot va tuong/cot doc theo quy dao tham chieu.
// Tuong chua biet duoc coi la co tuong (than trong).
float path_clearance(const Move* mv, int n, uint8_t start_cell, Dir heading, float start_offset, const Maze& m);

}  // namespace mm

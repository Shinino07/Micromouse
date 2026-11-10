#include "moves.h"
#include <cmath>
#include <initializer_list>

namespace mm {

TurnGeom g_turn[T_COUNT];
// chon theo khe ho lon nhat toi cot (sim --clear): S90 F90 F180 IN45 OUT45 IN135 OUT135 V90
float g_turn_frac[T_COUNT] = {1.0f, 0.8f, 1.0f, 1.0f, 1.0f, 0.75f, 0.75f, 0.7f};

namespace {
constexpr float PI = 3.14159265f;

// Duong cong don vi (dai 1): theta(u) = TH*(u - sin(2*pi*u)/(2*pi))
void unit_curve(float th, float& ex, float& ey, float& xmax) {
  const int n = 2000;
  double x = 0, y = 0, xm = 0;
  for (int i = 0; i < n; i++) {
    const double u = (i + 0.5) / n;
    const double a = th * (u - std::sin(2 * PI * u) / (2 * PI));
    x += std::cos(a) / n;
    y += std::sin(a) / n;
    if (x > xm) xm = x;
  }
  ex = static_cast<float>(x); ey = static_cast<float>(y); xmax = static_cast<float>(xm);
}

void make_turn(TurnType t, float fwd, float lat, float theta) {
  TurnGeom& g = g_turn[t];
  float cx, cy, xm;
  unit_curve(theta, cx, cy, xm);
  g.theta = theta; g.end_fwd = fwd; g.end_lat = lat;
  if (t == T_FAST180) {
    // lat = L*cy; dinh cung nam tren duong tam cua o ke (cach diem bat dau 180 mm)
    g.curve = lat / cy;
    g.pre = g.post = std::fmax(0.0f, cfg::CELL - g.curve * xm);
  } else {
    // end = pre*(1,0) + L*(cx,cy) + post*(cos,sin); chon L lon nhat ma pre, post >= 0
    const float s = std::sin(theta), c = std::cos(theta);
    const float a2 = lat / s, b2 = cy / s;                      // post = a2 - b2*L
    const float a1 = fwd - c * a2, b1 = cx - c * b2;            // pre  = a1 - b1*L
    float L = 1e9f;
    if (b1 > 1e-6f) L = std::fmin(L, a1 / b1);
    if (b2 > 1e-6f) L = std::fmin(L, a2 / b2);
    L *= g_turn_frac[t];
    g.curve = L;
    g.post = std::fmax(0.0f, a2 - b2 * L);
    g.pre = std::fmax(0.0f, a1 - b1 * L);
  }
  g.kmax = 2.0f * theta / g.curve;
}
}  // namespace

void init_turns() {
  const float H = cfg::HALF, C = cfg::CELL;
  const float r2 = 0.70710678f;
  make_turn(T_SEARCH90, H, H, PI / 2);
  make_turn(T_FAST90, C, C, PI / 2);
  make_turn(T_FAST180, 0.0f, C, PI);
  make_turn(T_IN45, C, H, PI / 4);
  make_turn(T_OUT45, (H + C) * r2, (C - H) * r2, PI / 4);
  make_turn(T_IN135, H, C, 3 * PI / 4);
  make_turn(T_OUT135, (C - H) * r2, (C + H) * r2, 3 * PI / 4);
  make_turn(T_V90, C * r2, C * r2, PI / 2);
}

float turn_curvature(const TurnGeom& g, float s) {
  const float u = (s - g.pre) / g.curve;
  if (u <= 0.0f || u >= 1.0f) return 0.0f;
  const float sn = std::sin(PI * u);
  return g.kmax * sn * sn;
}

float turn_speed(TurnType t, const cfg::RunParams& rp) {
  const float k = g_turn[t].kmax;
  float v = std::sqrt(rp.a_lat / k);
  // banh ngoai chay nhanh hon tam robot
  v = std::fmin(v, 0.9f * cfg::V_WHEEL_MAX / (1.0f + k * 0.5f * cfg::TRACK_MM));
  return std::fmin(v, t == T_SEARCH90 ? rp.v_search : rp.v_turn_cap);
}

float straight_vmax(float v) { return std::fmin(v, 0.92f * cfg::V_WHEEL_MAX); }

Move mk_straight(float dist, const cfg::RunParams& rp, float v_end, uint8_t flags, float edge_phase) {
  Move m;
  m.type = MoveType::STRAIGHT;
  m.dist = dist;
  m.v_max = straight_vmax(rp.v_max);
  m.v_end = v_end;
  m.accel = rp.accel;
  m.decel = rp.decel;
  m.flags = flags;
  m.edge_phase = edge_phase;
  return m;
}

Move mk_turn(TurnType t, int8_t dir, const cfg::RunParams& rp) {
  Move m;
  m.type = MoveType::TURN;
  m.turn = t;
  m.dir = dir;
  m.v_max = m.v_end = turn_speed(t, rp);
  m.accel = rp.accel;
  m.decel = rp.decel;
  m.dist = g_turn[t].pre + g_turn[t].curve + g_turn[t].post;
  return m;
}

Move mk_pivot(float angle_rad) {
  Move m;
  m.type = MoveType::PIVOT;
  m.dist = angle_rad;
  return m;
}

Move mk_front_align() {
  Move m;
  m.type = MoveType::FRONT_ALIGN;
  m.dist = cfg::FRONT_ALIGN_S;
  return m;
}

Move mk_wait(float seconds) {
  Move m;
  m.type = MoveType::WAIT;
  m.dist = seconds;
  return m;
}

namespace {
enum class Node : uint8_t { CENTER, EDGE, DIAG };
inline Act opp(Act a) { return a == ACT_L ? ACT_R : ACT_L; }
inline int8_t sgn(Act a) { return a == ACT_L ? 1 : -1; }
}  // namespace

int compile_route(const Route& r, Dir heading, float start_offset, const cfg::RunParams& rp,
                  Move* out, int max_moves) {
  int n_out = 0;
  bool ok = true;
  auto emit = [&](const Move& m) {
    if (n_out < max_moves) out[n_out++] = m; else ok = false;
  };

  if (r.first_dir != heading) {
    const int k = (r.first_dir - heading + 4) & 3;
    emit(mk_pivot(k == 1 ? -PI / 2 : (k == 3 ? PI / 2 : PI)));
    start_offset = 0;
  }

  auto tok = [&](int i) -> Act { return i < r.n ? r.act[i] : ACT_S; };
  const int n = r.n;
  Node node = Node::CENTER;
  float node_off = start_offset;
  float acc = 0, acc_phase = 0;
  int dacc = 0;

  auto acc_add = [&](float mm) {
    if (acc == 0) {
      acc_phase = (node == Node::CENTER ? cfg::HALF : 0.0f) - node_off;
      acc = -node_off;
      node_off = 0;
    }
    acc += mm;
  };
  auto flush = [&]() {
    if (acc > 0.5f) emit(mk_straight(acc, rp, 0, F_WALL | F_EDGE, acc_phase));
    acc = 0;
  };
  auto flush_diag = [&]() {
    if (dacc > 0) {
      Move m = mk_straight(dacc * cfg::DIAG_STEP, rp, 0, 0, -1);
      m.type = MoveType::DIAG;
      m.v_max = straight_vmax(rp.v_diag);
      emit(m);
    }
    dacc = 0;
  };

  // Cua toc do cao chi khi da co da (khong phai cua ngay tu luc dung yen)
  // va con it nhat 1 o phia sau de phanh
  auto moving = [&]() {
    if (acc + node_off > 40.0f) return true;
    return n_out > 0 && out[n_out - 1].type != MoveType::PIVOT;
  };

  int i = 1;
  while (i < n && ok) {
    const Act a = tok(i);
    if (node == Node::CENTER) {
      if (a == ACT_S) { acc_add(cfg::CELL); i++; continue; }
      const Act a1 = tok(i + 1);
      const bool fast_ok = moving();
      if (fast_ok && a1 == ACT_S && rp.big_turns && i + 2 < n) {
        flush(); emit(mk_turn(T_FAST90, sgn(a), rp)); i += 2; continue;
      }
      if (fast_ok && a1 == opp(a) && rp.diagonal) {
        flush(); emit(mk_turn(T_IN45, sgn(a), rp)); i += 1; node = Node::DIAG; continue;
      }
      if (fast_ok && a1 == a) {
        const Act a2 = tok(i + 2);
        if (a2 == ACT_S && rp.big_turns && i + 3 < n) {
          flush(); emit(mk_turn(T_FAST180, sgn(a), rp)); i += 3; continue;
        }
        if (a2 == opp(a) && rp.diagonal && i + 2 < n) {
          flush(); emit(mk_turn(T_IN135, sgn(a), rp)); i += 2; node = Node::DIAG; continue;
        }
      }
      acc_add(cfg::HALF);
      node = Node::EDGE;
      continue;
    }
    if (node == Node::EDGE) {
      if (a == ACT_S) { acc_add(cfg::HALF); node = Node::CENTER; i++; continue; }
      flush();
      emit(mk_turn(T_SEARCH90, sgn(a), rp));
      i++;
      continue;
    }
    // DIAG: dang vao o i theo huong cheo
    if (a == ACT_S) { ok = false; break; }
    const Act a1 = tok(i + 1);
    if (a1 == opp(a) && i + 1 < n) { dacc++; i++; continue; }
    if (a1 == ACT_S) {
      if (i + 2 >= n) { ok = false; break; }
      flush_diag(); emit(mk_turn(T_OUT45, sgn(a), rp)); i += 2; node = Node::CENTER; continue;
    }
    const Act a2 = tok(i + 2);
    if (a2 == ACT_S) {
      if (i + 3 >= n) { ok = false; break; }
      flush_diag(); emit(mk_turn(T_OUT135, sgn(a), rp)); i += 3; node = Node::CENTER; continue;
    }
    if (a2 == opp(a) && i + 2 < n) { flush_diag(); emit(mk_turn(T_V90, sgn(a), rp)); i += 2; continue; }
    ok = false;
  }
  if (!ok || node != Node::CENTER) return -1;
  flush();
  if (n_out == 0 || out[n_out - 1].type != MoveType::STRAIGHT) return -1;
  plan_speeds(out, n_out);
  for (int k = 0; k < n_out; k++)
    if (out[k].type == MoveType::TURN && out[k].v_max < 50.0f) return -1;
  return n_out;
}

void plan_speeds(Move* mv, int n) {
  float next_entry = 0.0f;
  for (int i = n - 1; i >= 0; i--) {
    Move& m = mv[i];
    switch (m.type) {
      case MoveType::STRAIGHT:
      case MoveType::DIAG:
        m.v_end = std::fmin(next_entry, m.v_max);
        next_entry = std::fmin(m.v_max, std::sqrt(m.v_end * m.v_end + 2.0f * m.decel * m.dist));
        break;
      case MoveType::TURN: {
        const float v = std::fmin(m.v_max, next_entry);
        m.v_max = m.v_end = v;
        next_entry = v;
        break;
      }
      default:
        m.v_end = 0;
        next_entry = 0;
        break;
    }
  }
}

bool verify_moves(const Move* mv, int n, uint8_t start_cell, Dir heading, float start_offset,
                  uint8_t end_cell, Dir end_heading) {
  double th = heading_rad(heading);
  double x = (cell_x(start_cell) + 0.5) * cfg::CELL - std::sin(th) * start_offset;
  double y = (cell_y(start_cell) + 0.5) * cfg::CELL + std::cos(th) * start_offset;
  for (int i = 0; i < n; i++) {
    const Move& m = mv[i];
    const double hx = -std::sin(th), hy = std::cos(th);  // huong tien (NORTH = +y)
    if (m.type == MoveType::STRAIGHT || m.type == MoveType::DIAG) {
      x += hx * m.dist; y += hy * m.dist;
    } else if (m.type == MoveType::TURN) {
      const TurnGeom& g = g_turn[m.turn];
      const double lx = -hy * m.dir, ly = hx * m.dir;  // vector ve phia re
      x += hx * g.end_fwd + lx * g.end_lat;
      y += hy * g.end_fwd + ly * g.end_lat;
      th += m.dir * g.theta;
    } else if (m.type == MoveType::PIVOT) {
      th += m.dist;
    }
  }
  const double ex = (cell_x(end_cell) + 0.5) * cfg::CELL, ey = (cell_y(end_cell) + 0.5) * cfg::CELL;
  double dth = std::remainder(th - heading_rad(end_heading), 2 * PI);
  return std::fabs(x - ex) < 1.0 && std::fabs(y - ey) < 1.0 && std::fabs(dth) < 1e-3;
}

namespace {
// khoang cach tu diem toi hinh chu nhat [x0,x1]x[y0,y1] (0 neu nam trong)
inline float rect_dist(float px, float py, float x0, float y0, float x1, float y1) {
  const float dx = std::fmax(std::fmax(x0 - px, 0.0f), px - x1);
  const float dy = std::fmax(std::fmax(y0 - py, 0.0f), py - y1);
  return std::sqrt(dx * dx + dy * dy);
}

float point_clearance(float px, float py, const Maze& m) {
  const float C = cfg::CELL, W = cfg::WALL_HALF;
  int cx = static_cast<int>(std::floor(px / C)), cy = static_cast<int>(std::floor(py / C));
  if (!in_maze(cx, cy)) return 0.0f;
  float best = 1e9f;
  const float x0 = cx * C, y0 = cy * C, x1 = x0 + C, y1 = y0 + C;
  for (int d = 0; d < 4; d++) {
    const Dir dd = static_cast<Dir>(d);
    if (!m.wall(cx, cy, dd) && m.known(cx, cy, dd)) continue;
    float d0;
    if (dd == NORTH) d0 = rect_dist(px, py, x0 - W, y1 - W, x1 + W, y1 + W);
    else if (dd == SOUTH) d0 = rect_dist(px, py, x0 - W, y0 - W, x1 + W, y0 + W);
    else if (dd == EAST) d0 = rect_dist(px, py, x1 - W, y0 - W, x1 + W, y1 + W);
    else d0 = rect_dist(px, py, x0 - W, y0 - W, x0 + W, y1 + W);
    best = std::fmin(best, d0);
  }
  const float cxs[2] = {x0, x1}, cys[2] = {y0, y1};
  for (float qx : cxs)
    for (float qy : cys) best = std::fmin(best, rect_dist(px, py, qx - W, qy - W, qx + W, qy + W));
  return best;
}

float footprint_clearance(float x, float y, float th, const Maze& m) {
  const float hx = -std::sin(th), hy = std::cos(th), lx = -hy, ly = hx;
  float best = 1e9f;
  const float F = cfg::ROBOT_FRONT, B = cfg::ROBOT_BACK, Wd = cfg::ROBOT_HALF_WIDTH;
  for (float f = -B; f <= F + 0.1f; f += (F + B) / 6.0f)
    for (float l : {Wd, -Wd}) best = std::fmin(best, point_clearance(x + hx * f + lx * l, y + hy * f + ly * l, m));
  for (float l = -Wd; l <= Wd + 0.1f; l += Wd / 2.0f) {
    best = std::fmin(best, point_clearance(x + hx * F + lx * l, y + hy * F + ly * l, m));
    best = std::fmin(best, point_clearance(x - hx * B + lx * l, y - hy * B + ly * l, m));
  }
  return best;
}
}  // namespace

float path_clearance(const Move* mv, int n, uint8_t start_cell, Dir heading, float start_offset, const Maze& m) {
  float th = heading_rad(heading);
  float x = (cell_x(start_cell) + 0.5f) * cfg::CELL - std::sin(th) * start_offset;
  float y = (cell_y(start_cell) + 0.5f) * cfg::CELL + std::cos(th) * start_offset;
  float best = footprint_clearance(x, y, th, m);
  constexpr float DS = 4.0f;
  for (int i = 0; i < n; i++) {
    const Move& mo = mv[i];
    if (mo.type == MoveType::PIVOT) {
      const int steps = static_cast<int>(std::fabs(mo.dist) / 0.1f) + 1;
      for (int k = 0; k < steps; k++) {
        th += mo.dist / steps;
        best = std::fmin(best, footprint_clearance(x, y, th, m));
      }
      continue;
    }
    if (mo.type != MoveType::STRAIGHT && mo.type != MoveType::DIAG && mo.type != MoveType::TURN) continue;
    const TurnGeom* g = mo.type == MoveType::TURN ? &g_turn[mo.turn] : nullptr;
    const int steps = static_cast<int>(mo.dist / DS) + 1;
    const float ds = mo.dist / steps;
    for (int k = 0; k < steps; k++) {
      const float s_mid = (k + 0.5f) * ds;
      const float th_mid = g ? th + 0.5f * mo.dir * turn_curvature(*g, s_mid) * ds : th;
      x += -std::sin(th_mid) * ds;
      y += std::cos(th_mid) * ds;
      if (g) th += mo.dir * turn_curvature(*g, s_mid) * ds;
      best = std::fmin(best, footprint_clearance(x, y, th, m));
    }
    if (g) th = std::round(th / (PI / 4)) * (PI / 4);
  }
  return best;
}

}  // namespace mm

// Mo phong toan bo: chay dung code core/ (Mouse + Motion) tren mo hinh vat ly + tia IR
//   sim.exe <maze.txt> [seed] [--quiet]      chay che do AUTO (3 luot) tren me cung
//   sim.exe --random <so_me_cung> [seed]     kiem tra trinh lap duong tren me cung ngau nhien
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "../core/hw.h"
#include "../core/mouse.h"

using namespace mm;

namespace {
constexpr double PI = 3.14159265358979;

// ---------------- me cung that ----------------
struct TrueMaze {
  bool h[cfg::N + 1][cfg::N] = {};  // tuong ngang tai duong luoi y, o cot x
  bool v[cfg::N][cfg::N + 1] = {};  // tuong doc tai duong luoi x, hang y
  bool wall(int x, int y, Dir d) const {
    switch (d) {
      case NORTH: return h[y + 1][x];
      case SOUTH: return h[y][x];
      case EAST: return v[y][x + 1];
      default: return v[y][x];
    }
  }
};

bool load_maze_txt(const char* path, TrueMaze& t) {
  std::ifstream f(path);
  if (!f) return false;
  std::vector<std::string> L;
  std::string s;
  while (std::getline(f, s)) { if (!s.empty() && s.back() == '\r') s.pop_back(); if (!s.empty()) L.push_back(s); }
  if (L.size() != 2 * cfg::N + 1) return false;
  for (int gy = cfg::N; gy >= 0; gy--) {
    const std::string& row = L[2 * (cfg::N - gy)];
    for (int x = 0; x < cfg::N; x++) t.h[gy][x] = row.size() > size_t(4 * x + 2) && row[4 * x + 2] == '-';
    if (gy == 0) break;
    const std::string& vr = L[2 * (cfg::N - gy) + 1];
    for (int x = 0; x <= cfg::N; x++) t.v[gy - 1][x] = vr.size() > size_t(4 * x) && vr[4 * x] == '|';
  }
  return true;
}

void random_maze(TrueMaze& t, std::mt19937& rng) {
  for (auto& r : t.h) for (auto& b : r) b = true;
  for (auto& r : t.v) for (auto& b : r) b = true;
  bool seen[cfg::N][cfg::N] = {};
  std::vector<std::pair<int, int>> st{{0, 0}};
  seen[0][0] = true;
  auto knock = [&](int x, int y, int d) {
    if (d == 0) t.h[y + 1][x] = false; else if (d == 2) t.h[y][x] = false;
    else if (d == 1) t.v[y][x + 1] = false; else t.v[y][x] = false;
  };
  while (!st.empty()) {
    auto [x, y] = st.back();
    int opts[4], n = 0;
    for (int d = 0; d < 4; d++) {
      const int nx = x + DX[d], ny = y + DY[d];
      if (in_maze(nx, ny) && !seen[ny][nx]) opts[n++] = d;
    }
    if (!n) { st.pop_back(); continue; }
    const int d = opts[rng() % n];
    knock(x, y, d);
    seen[y + DY[d]][x + DX[d]] = true;
    st.push_back({x + DX[d], y + DY[d]});
  }
  for (int k = 0; k < 40; k++) {  // them vong de co nhieu duong
    const int x = 1 + rng() % (cfg::N - 2), y = 1 + rng() % (cfg::N - 2);
    knock(x, y, rng() % 4);
  }
  const int g = cfg::GOAL_X0;
  t.v[g][g + 1] = t.v[g + 1][g + 1] = false;
  t.h[g + 1][g] = t.h[g + 1][g + 1] = false;
  t.v[0][1] = true;
  t.h[1][0] = false;
}

// ---------------- hinh hoc tuong (hinh chu nhat) ----------------
struct Rect { double x0, y0, x1, y1; };
std::vector<Rect> g_rects[cfg::N][cfg::N];  // tuong + cot quanh moi o

void build_rects(const TrueMaze& t) {
  const double C = cfg::CELL, W = cfg::WALL_HALF;
  std::vector<Rect> all;
  for (int gy = 0; gy <= cfg::N; gy++)
    for (int x = 0; x < cfg::N; x++)
      if (t.h[gy][x]) all.push_back({x * C - W, gy * C - W, (x + 1) * C + W, gy * C + W});
  for (int y = 0; y < cfg::N; y++)
    for (int gx = 0; gx <= cfg::N; gx++)
      if (t.v[y][gx]) all.push_back({gx * C - W, y * C - W, gx * C + W, (y + 1) * C + W});
  for (int gy = 0; gy <= cfg::N; gy++)
    for (int gx = 0; gx <= cfg::N; gx++) all.push_back({gx * C - W, gy * C - W, gx * C + W, gy * C + W});
  for (int y = 0; y < cfg::N; y++)
    for (int x = 0; x < cfg::N; x++) {
      g_rects[y][x].clear();
      const double ax = (x - 1.5) * C, ay = (y - 1.5) * C, bx = (x + 2.5) * C, by = (y + 2.5) * C;
      for (const Rect& r : all)
        if (r.x1 >= ax && r.x0 <= bx && r.y1 >= ay && r.y0 <= by) g_rects[y][x].push_back(r);
    }
}

const std::vector<Rect>& rects_near(double x, double y) {
  int cx = static_cast<int>(x / cfg::CELL), cy = static_cast<int>(y / cfg::CELL);
  cx = cx < 0 ? 0 : (cx >= cfg::N ? cfg::N - 1 : cx);
  cy = cy < 0 ? 0 : (cy >= cfg::N ? cfg::N - 1 : cy);
  return g_rects[cy][cx];
}

double raycast(double ox, double oy, double dx, double dy, double maxd) {
  double best = maxd;
  for (const Rect& r : rects_near(ox, oy)) {
    double t0 = 0, t1 = best;
    const double o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {r.x0, r.y0}, hi[2] = {r.x1, r.y1};
    bool hit = true;
    for (int k = 0; k < 2 && hit; k++) {
      if (std::fabs(d[k]) < 1e-12) { if (o[k] < lo[k] || o[k] > hi[k]) hit = false; continue; }
      double a = (lo[k] - o[k]) / d[k], b = (hi[k] - o[k]) / d[k];
      if (a > b) std::swap(a, b);
      if (a > t0) t0 = a;
      if (b < t1) t1 = b;
      if (t0 > t1) hit = false;
    }
    if (hit && t0 < best) best = t0;
  }
  return best;
}

bool inside_wall(double x, double y) {
  for (const Rect& r : rects_near(x, y))
    if (x > r.x0 && x < r.x1 && y > r.y0 && y < r.y1) return true;
  return false;
}

// ---------------- robot vat ly ----------------
struct Robot {
  double x, y, th;           // th = 0: huong BAC, duong = quay trai
  double vl = 0, vr = 0;     // mm/s
  double pl = 0, pr = 0;     // quang duong banh
  double gyro_bias = 0.004;  // rad/s
};

uint32_t g_ms = 0;
bool g_quiet = false;
Maze g_flash;
bool g_flash_ok = false;
}  // namespace

// ---------------- dich vu hw cho core ----------------
namespace mm::hw {
uint32_t millis() { return g_ms; }
bool btn_mode() { return false; }
bool btn_start() { return false; }
bool load_maze(Maze& m) { if (g_flash_ok) m = g_flash; return g_flash_ok; }
void save_maze(const Maze& m) { g_flash = m; g_flash_ok = true; }
void erase_maze() { g_flash_ok = false; }
void system_reset() { std::printf("[sim] system reset requested\n"); }
void log(const char* fmt, ...) {
  if (g_quiet) return;
  std::printf("[%7.3f] ", g_ms / 1000.0);
  va_list ap;
  va_start(ap, fmt);
  std::vprintf(fmt, ap);
  va_end(ap);
}
}  // namespace mm::hw

namespace {

struct SimResult {
  uint32_t goal[4] = {};
  int runs = 0;
  int collisions = 0;
  bool fault = false;
  bool finished = false;
  int wrong_walls = 0;
  double min_clear = 1e9;
};

int compare_maze(const Maze& m, const TrueMaze& t) {
  int bad = 0;
  for (int y = 0; y < cfg::N; y++)
    for (int x = 0; x < cfg::N; x++)
      for (int d = 0; d < 4; d++)
        if (m.known(x, y, static_cast<Dir>(d)) && m.wall(x, y, static_cast<Dir>(d)) != t.wall(x, y, static_cast<Dir>(d))) {
          bad++;
          if (!g_quiet) std::printf("  sai tuong o (%d,%d) huong %d: robot=%d that=%d\n", x, y, d,
                                    m.wall(x, y, static_cast<Dir>(d)), t.wall(x, y, static_cast<Dir>(d)));
        }
  return bad / 2;
}

SimResult simulate(const TrueMaze& tm, uint32_t seed, double max_s) {
  build_rects(tm);
  std::mt19937 rng(seed);
  std::normal_distribution<double> nrm(0.0, 1.0);
  g_ms = 0;
  g_flash_ok = false;

  static Mouse* mp = nullptr;
  delete mp;
  mp = new Mouse();
  Mouse& mouse = *mp;
  mouse.init();

  Robot rb;
  rb.x = 0.5 * cfg::CELL;
  rb.y = 0.5 * cfg::CELL + cfg::START_OFFSET;
  rb.th = 0;
  // sai lech mo hinh so voi tham so trong config (robot that khong bao gio khop)
  const double kv_err[2] = {1.0 + 0.06 * nrm(rng), 1.0 + 0.06 * nrm(rng)};
  const double ka_err = 1.0 + 0.10 * nrm(rng);
  const double ri_err = 1.0 + 0.15 * nrm(rng);
  const double wheel_err[2] = {1.0 + 0.004 * nrm(rng), 1.0 + 0.004 * nrm(rng)};

  float ir[cfg::IR_COUNT];
  for (auto& v : ir) v = cfg::IR_NONE;
  SimResult res;
  bool started = false;
  const double og_dummy = 0;
  int collide_ticks = 0;
  const double dt = cfg::DT_S;

  for (uint64_t tick = 0; tick < static_cast<uint64_t>(max_s * 1000); tick++) {
    g_ms = static_cast<uint32_t>(tick);
    // IR: moi tick cap nhat 1 nhom (giong port STM32)
    const int grp = tick % 3;
    for (int i = 0; i < cfg::IR_COUNT; i++) {
      const int gi = i == cfg::IR_F ? 0 : ((i == cfg::IR_L || i == cfg::IR_R) ? 1 : 2);   // truoc / ben / cheo
      if (gi != grp) continue;
      const cfg::IrPose& p = cfg::IR_POSE[i];
      const double hx = -std::sin(rb.th), hy = std::cos(rb.th);
      const double lx = -hy, ly = hx;
      const double ox = rb.x + hx * p.fwd + lx * p.lat, oy = rb.y + hy * p.fwd + ly * p.lat;
      const double a = rb.th + p.ang;
      double d = raycast(ox, oy, -std::sin(a), std::cos(a), 260.0);
      if (d >= 250.0) ir[i] = cfg::IR_NONE;
      else ir[i] = static_cast<float>(d * (1.0 + 0.015 * nrm(rng)) + 0.5 * nrm(rng));
      // mat that bao hoa khi qua gan: doc ~ nguong bao hoa (dung im) chu khong phai khoang cach that
      if (ir[i] < cfg::IR_SAT_MM[i]) ir[i] = cfg::IR_SAT_MM[i] - 1.0f + 0.3f * static_cast<float>(nrm(rng));
    }

    TickIn in;
    // encoder that dem nguoc theo ENC_SIGN (banh phai dem am khi tien) + luong tu hoa 1 xung
    in.enc_l = cfg::ENC_SIGN_L * static_cast<int32_t>(std::floor(rb.pl * cfg::COUNTS_PER_MM_L));
    in.enc_r = cfg::ENC_SIGN_R * static_cast<int32_t>(std::floor(rb.pr * cfg::COUNTS_PER_MM_R));
    const double w_true = (rb.vr - rb.vl) / cfg::TRACK_MM;
    // gyro that lap nguoc (GYRO_SIGN) va lech he so (GYRO_SCALE): controller tinh w = gyro * SIGN * SCALE
    in.gyro_z = static_cast<float>(cfg::GYRO_SIGN * w_true / cfg::GYRO_SCALE + rb.gyro_bias + 0.01 * nrm(rng));
    for (int i = 0; i < cfg::IR_COUNT; i++) in.ir[i] = ir[i];
    in.vbat = static_cast<float>(cfg::VBAT_NOMINAL * (1.0 - 0.05 * tick / (max_s * 1000)));
    TickOut out;
    mouse.tick(in, out);

    // dong co: V = KV*v*(sai lech) + KA*a + KS*sign
    // tach thanh phan tinh tien / xoay: quan tinh xoay = ROT_INERTIA (co sai lech)
    const double duty[2] = {out.duty_l * cfg::MOTOR_SIGN_L, out.duty_r * cfg::MOTOR_SIGN_R};
    const double vv[2] = {rb.vl, rb.vr};
    double F[2];
    for (int k = 0; k < 2; k++) {
      const double V = duty[k] * in.vbat;
      double fr = std::fabs(vv[k]) > 1 ? cfg::KS_MOTOR * (vv[k] > 0 ? 1 : -1)
                                       : (std::fabs(V) < cfg::KS_MOTOR ? V : cfg::KS_MOTOR * (V > 0 ? 1 : -1));
      F[k] = V - cfg::KV_MOTOR * kv_err[k] * vv[k] - fr;
    }
    const double ac = 0.5 * (F[0] + F[1]) / (cfg::KA_MOTOR * ka_err);
    const double ad = 0.5 * (F[1] - F[0]) / (cfg::KA_MOTOR * ka_err * cfg::ROT_INERTIA * ri_err);
    rb.vl += (ac - ad) * dt;
    rb.vr += (ac + ad) * dt;
    if (std::getenv("SIM_IDEAL")) {
      const double half = 0.5 * cfg::TRACK_MM;
      rb.vl = mouse.motion().v_ref() - mouse.motion().w_ref() * half;
      rb.vr = mouse.motion().v_ref() + mouse.motion().w_ref() * half;
    }
    rb.pl += rb.vl * dt * wheel_err[0];
    rb.pr += rb.vr * dt * wheel_err[1];
    const double v = 0.5 * (rb.vl + rb.vr), w = (rb.vr - rb.vl) / cfg::TRACK_MM;
    rb.th += w * dt;
    rb.x += -std::sin(rb.th) * v * dt;
    rb.y += std::cos(rb.th) * v * dt;

    // va cham: kiem tra vien robot
    bool hit = false;
    const double hx = -std::sin(rb.th), hy = std::cos(rb.th), lx = -hy, ly = hx;
    for (double f : {cfg::ROBOT_FRONT * 1.0, 0.0, -cfg::ROBOT_BACK * 1.0})
      for (double l : {cfg::ROBOT_HALF_WIDTH * 1.0, -cfg::ROBOT_HALF_WIDTH * 1.0, 0.0}) {
        if (f == 0.0 && l == 0.0) continue;
        if (inside_wall(rb.x + hx * f + lx * l, rb.y + hy * f + ly * l)) hit = true;
      }
    if (tick % 5 == 0) {
      // khoang ho nho nhat tu vien robot toi tuong / cot
      double cl = 1e9;
      for (double f = -cfg::ROBOT_BACK; f <= cfg::ROBOT_FRONT + 0.1; f += 10.0)
        for (double l : {cfg::ROBOT_HALF_WIDTH * 1.0, -cfg::ROBOT_HALF_WIDTH * 1.0}) {
          const double px = rb.x + hx * f + lx * l, py = rb.y + hy * f + ly * l;
          for (const Rect& r : rects_near(px, py)) {
            const double ddx = std::fmax(std::fmax(r.x0 - px, 0.0), px - r.x1);
            const double ddy = std::fmax(std::fmax(r.y0 - py, 0.0), py - r.y1);
            cl = std::fmin(cl, std::sqrt(ddx * ddx + ddy * ddy));
          }
        }
      if (mouse.runs_done() >= 1 && cl < res.min_clear) res.min_clear = cl;
    }
    if (hit) { if (collide_ticks++ == 0) res.collisions++; } else collide_ticks = 0;
    if (hit && !g_quiet && collide_ticks == 1)
      std::printf("[%7.3f] !!! VA CHAM tai (%.0f, %.0f) th=%.1f deg e_th=%.2f deg move %d/%d s=%.0f v=%.0f e_s=%.1f\n",
                  g_ms / 1000.0, rb.x, rb.y, std::remainder(rb.th, 2 * PI) * 180 / PI,
                  (mouse.motion().th_ref() - mouse.motion().heading()) * 180 / PI, mouse.motion().move_type(),
                  mouse.motion().move_turn(), mouse.motion().move_s(), mouse.motion().v_ref(), mouse.motion().pos_err());

    {
      static int prev_type = -2;
      const int mt = mouse.motion().move_type();
      if (std::getenv("SIM_TURNLOG") && mt == 2 && prev_type != 2) {
        const double th = std::remainder(rb.th, 2 * PI);
        const double q = std::round(th / (PI / 4)) * (PI / 4);
        const bool ortho = std::fmod(std::fabs(std::round(th / (PI / 4))), 2.0) < 0.5;
        double lat = 0;
        if (ortho) {
          const bool ns = std::fabs(std::sin(q)) < 0.5;
          lat = ns ? rb.x - (std::floor(rb.x / cfg::CELL) + 0.5) * cfg::CELL : rb.y - (std::floor(rb.y / cfg::CELL) + 0.5) * cfg::CELL;
        }
        std::printf("[%7.3f] turn %d start (%.1f, %.1f) th %.2f deg lat %.1f wall %.3f\n", g_ms / 1000.0,
                    mouse.motion().move_turn(), rb.x, rb.y, (th - q) * 180 / PI, lat, mouse.motion().wall_term_dbg());
      }
      if (std::getenv("SIM_TURNLOG")) {
        // lech ngang so voi duong cheo qua trung diem mep o (x -+ y = 90 mod 180)
        static double diag_max = 0;
        static bool in_diag = false;
        const double th = std::remainder(rb.th, 2 * PI);
        const double oct = std::round(th / (PI / 4));
        const bool diag = std::fmod(std::fabs(oct), 2.0) > 0.5;
        if (diag && (mt == 1 || mt == 2)) {
          const bool ne_sw = (oct == -1 || oct == 3 || oct == -3 + 0 * 1) ? false : true;
          const double v = std::fabs(std::sin(th) * std::cos(th)) > 0.3 ? (std::sin(th) * std::cos(th) < 0 ? rb.x - rb.y : rb.x + rb.y) : 0;
          (void)ne_sw;
          const double lat = std::remainder(v - 90.0, 180.0) / std::sqrt(2.0);
          if (!in_diag) { std::printf("[%7.3f] diag start lat %.1f (move %d)\n", g_ms / 1000.0, lat, mt); diag_max = 0; }
          if (mouse.motion().dbg_new_) {
            mouse.motion().dbg_new_ = false;
            const double hx = -std::sin(rb.th), hy = std::cos(rb.th), lx = -hy, ly = hx;
            const double proj = rb.x * lx + rb.y * ly;
            const double off = std::remainder(proj - 63.6396, 127.2792);
            std::printf("[%7.3f]   diag meas side %d e=%.1f  true move-left=%.1f\n", g_ms / 1000.0,
                        mouse.motion().dbg_side_, mouse.motion().dbg_e_, -off);
          }
          in_diag = true;
          if (std::fabs(lat) > std::fabs(diag_max)) diag_max = lat;
        } else if (in_diag) {
          std::printf("[%7.3f] diag end max lat %.1f\n", g_ms / 1000.0, diag_max);
          in_diag = false;
        }
      }
      prev_type = mt;
    }
    if (const char* at = std::getenv("SIM_ALIGN")) {
      // test can tuong: robot o tam o (0,0) quay mat NAM, lech (ngang, doc, goc do) = SIM_ALIGN
      if (tick == 0) {
        double ox = 0, oy = 0, og = 0;
        std::sscanf(at, "%lf,%lf,%lf", &ox, &oy, &og);
        rb.x = 90 + ox; rb.y = 90 + oy; rb.th = PI + og * PI / 180;
      }
      if (tick == 100) { mouse.motion().enable(true); mouse.motion().push(mk_front_align()); }
      if (tick % 50 == 0 && tick > 100 && tick < 700) std::printf("t %4d true %.2f raw %.2f FL %.1f FR %.1f\n", (int)tick, std::remainder(rb.th - PI, 2 * PI) * 180 / PI, mouse.motion().dbg_e_ * 180 / PI, mouse.motion().ir(0), mouse.motion().ir(1));
      if (tick == 1500) {
        const double true_g = std::remainder(rb.th - PI, 2 * PI);
        std::printf("sau can: lech x %.2f mm, y %.2f mm, goc that %.2f deg, goc gyro tin %.2f deg (sai %.2f)\n",
                    rb.x - 90, rb.y - 90, true_g * 180 / PI, mouse.motion().heading() * 180 / PI,
                    (mouse.motion().heading() - true_g) * 180 / PI + og_dummy);
        break;
      }
      continue;
    }
    mouse.loop();
    if (!started) { mouse.start(Mode::AUTO); started = true; }
    if (mouse.had_fault()) { res.fault = true; break; }
    if (started && tick > 2000 && mouse.idle()) { res.finished = true; break; }
  }
  for (int r = 0; r < 4; r++) res.goal[r] = mouse.goal_time_ms(r);
  res.runs = mouse.runs_done();
  res.wrong_walls = compare_maze(mouse.maze(), tm);
  if (!g_quiet)
    std::printf("ket thuc t=%.1f s, robot (%.0f, %.0f) th=%.1f deg, sai so vi tri max %.1f mm\n", g_ms / 1000.0, rb.x,
                rb.y, rb.th * 180 / PI, mouse.motion().max_pos_err());
  return res;
}

// Kiem tra trinh bien dich duong (khong vat ly): ban do biet het
int planner_test(int count, uint32_t seed) {
  std::mt19937 rng(seed);
  init_turns();
  int fails = 0, diag_used = 0;
  static Move mv[400];
  for (int k = 0; k < count; k++) {
    TrueMaze t;
    random_maze(t, rng);
    Maze m;
    m.reset();
    for (int y = 0; y < cfg::N; y++)
      for (int x = 0; x < cfg::N; x++)
        for (int d = 0; d < 4; d++) m.set_wall(x, y, static_cast<Dir>(d), t.wall(x, y, static_cast<Dir>(d)));
    Route r;
    if (!find_route(m, 0, NORTH, goal_mask(), false, r)) { fails++; continue; }
    // vong ve: tu dich ve o xuat phat
    for (int pass = 0; pass < 2; pass++) {
      Route rr = r;
      uint8_t end_cell = rr.cell[rr.n - 1];
      Dir start_h = NORTH;
      uint8_t start_c = 0;
      if (pass == 1) {
        start_c = end_cell;
        const int dx = cell_x(rr.cell[rr.n - 1]) - cell_x(rr.cell[rr.n - 2]);
        const int dy = cell_y(rr.cell[rr.n - 1]) - cell_y(rr.cell[rr.n - 2]);
        start_h = dx == 1 ? EAST : dx == -1 ? WEST : dy == 1 ? NORTH : SOUTH;
        if (!find_route(m, start_c, start_h, single_mask(0, 0), false, rr)) { fails++; continue; }
        end_cell = 0;
      }
      const int dx = cell_x(rr.cell[rr.n - 1]) - cell_x(rr.cell[rr.n - 2]);
      const int dy = cell_y(rr.cell[rr.n - 1]) - cell_y(rr.cell[rr.n - 2]);
      const Dir eh = dx == 1 ? EAST : dx == -1 ? WEST : dy == 1 ? NORTH : SOUTH;
      bool ok = false;
      for (int level = 0; level < 3 && !ok; level++) {
        cfg::RunParams p = cfg::RUN_FAST;
        if (level >= 1) p.diagonal = false;
        if (level >= 2) p.big_turns = false;
        const int n = compile_route(rr, start_h, pass == 0 ? cfg::START_OFFSET : 0.0f, p, mv, 400);
        ok = n > 0 && verify_moves(mv, n, start_c, start_h, pass == 0 ? cfg::START_OFFSET : 0.0f, end_cell, eh);
        if (ok && level == 0) diag_used++;
      }
      if (!ok) fails++;
    }
  }
  std::printf("planner: %d me cung x 2 chieu, loi %d, dung duoc cheo %d\n", count, fails, diag_used);
  return fails;
}

}  // namespace

// Khe ho tung move cua duong nhanh nhat tren me cung da biet het
void clearance_report(const TrueMaze& t) {
  init_turns();
  Maze m;
  m.reset();
  for (int y = 0; y < cfg::N; y++)
    for (int x = 0; x < cfg::N; x++)
      for (int d = 0; d < 4; d++) m.set_wall(x, y, static_cast<Dir>(d), t.wall(x, y, static_cast<Dir>(d)));
  Route r;
  if (!find_route(m, 0, NORTH, goal_mask(), false, r)) return;
  static Move mv[400];
  const int n = compile_route(r, NORTH, 0.0f, cfg::RUN_FAST, mv, 400);
  static const char* TN[] = {"S90", "F90", "F180", "IN45", "OUT45", "IN135", "OUT135", "V90"};
  float prev = 1e9f;
  for (int k = 1; k <= n; k++) {
    const float c = path_clearance(mv, k, 0, NORTH, 0.0f, m);
    const Move& mo = mv[k - 1];
    if (c < prev - 0.01f)
      std::printf("move %2d %-7s dir %+d dist %6.1f v %5.0f -> khe ho %.1f\n", k - 1,
                  mo.type == MoveType::TURN ? TN[mo.turn] : (mo.type == MoveType::DIAG ? "DIAG" : "STR"), mo.dir,
                  mo.dist, mo.v_max, c);
    prev = c;
  }
  for (int t2 = 0; t2 < T_COUNT; t2++)
    std::printf("%-7s pre %6.1f curve %6.1f post %6.1f kmax %.4f v_fast %.0f\n", TN[t2], g_turn[t2].pre, g_turn[t2].curve,
                g_turn[t2].post, g_turn[t2].kmax, turn_speed(static_cast<TurnType>(t2), cfg::RUN_FAST));
}

int main(int argc, char** argv) {
  if (argc >= 3 && std::strcmp(argv[1], "--clear") == 0) {
    TrueMaze t;
    if (!load_maze_txt(argv[2], t)) return 2;
    clearance_report(t);
    return 0;
  }
  if (argc >= 3 && std::strcmp(argv[1], "--random") == 0) {
    const int n = std::atoi(argv[2]);
    const uint32_t seed = argc >= 4 ? static_cast<uint32_t>(std::atoi(argv[3])) : 1;
    int bad = planner_test(n, seed);
    // vat ly tren vai me cung ngau nhien
    g_quiet = true;
    std::mt19937 rng(seed + 99);
    int phys_bad = 0;
    const int phys_n = n < 20 ? n : 20;
    for (int k = 0; k < phys_n; k++) {
      TrueMaze t;
      random_maze(t, rng);
      if (std::getenv("SIM_DUMP")) {
        char fn[64];
        std::snprintf(fn, sizeof fn, "%s/rand_%d.txt", std::getenv("SIM_DUMP"), k);
        FILE* f = std::fopen(fn, "w");
        for (int gy = cfg::N; gy >= 0; gy--) {
          std::fputc('o', f);
          for (int x = 0; x < cfg::N; x++) std::fputs(t.h[gy][x] ? "---o" : "   o", f);
          std::fputc(10, f);
          if (gy == 0) break;
          for (int x = 0; x <= cfg::N; x++) { std::fputc(t.v[gy - 1][x] ? '|' : ' ', f); if (x < cfg::N) std::fputs("   ", f); }
          std::fputc(10, f);
        }
        std::fclose(f);
      }
      SimResult r = simulate(t, seed + k, 600);
      const bool ok = r.finished && !r.fault && r.collisions == 0 && r.wrong_walls == 0 && r.runs == 3;
      if (!ok) phys_bad++;
      std::printf("  maze %2d: runs %d goal %.2f / %.2f / %.2f s, va cham %d, sai tuong %d%s\n", k, r.runs,
                  r.goal[0] / 1000.0, r.goal[1] / 1000.0, r.goal[2] / 1000.0, r.collisions, r.wrong_walls,
                  ok ? "" : (r.fault ? "  FAULT" : "  LOI"));
    }
    std::printf("vat ly: %d/%d me cung loi\n", phys_bad, phys_n);
    return bad || phys_bad;
  }
  if (argc < 2) { std::printf("dung: sim <maze.txt> [seed] [--quiet]\n"); return 2; }
  TrueMaze t;
  if (!load_maze_txt(argv[1], t)) { std::printf("khong doc duoc %s\n", argv[1]); return 2; }
  const uint32_t seed = argc >= 3 && argv[2][0] != '-' ? static_cast<uint32_t>(std::atoi(argv[2])) : 1;
  for (int i = 2; i < argc; i++) if (std::strcmp(argv[i], "--quiet") == 0) g_quiet = true;
  SimResult r = simulate(t, seed, 600);
  std::printf("khe ho nho nhat trong luot nhanh: %.1f mm\n", r.min_clear);
  std::printf("\nKET QUA: %d luot, thoi gian toi dich: %.2f s | %.2f s | %.2f s, va cham %d, sai tuong %d, %s\n", r.runs,
              r.goal[0] / 1000.0, r.goal[1] / 1000.0, r.goal[2] / 1000.0, r.collisions, r.wrong_walls,
              r.fault ? "FAULT" : (r.finished ? "hoan tat" : "het gio"));
  return (r.fault || !r.finished || r.collisions) ? 1 : 0;
}

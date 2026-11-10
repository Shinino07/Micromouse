#include "mouse.h"
#include <cmath>
#include <initializer_list>
#include "hw.h"

namespace mm {

namespace {
constexpr float PI = 3.14159265f;
constexpr uint8_t START_CELL = 0;

bool fixed_wall(int x, int y, Dir d) {
  const int nx = x + DX[d], ny = y + DY[d];
  if (!in_maze(nx, ny)) return true;
  if (is_goal(x, y) && is_goal(nx, ny)) return true;
  return (x == 0 && y == 0) || (nx == 0 && ny == 0);
}

Dir dir_between(uint8_t a, uint8_t b) {
  const int dx = cell_x(b) - cell_x(a), dy = cell_y(b) - cell_y(a);
  if (dx == 1) return EAST;
  if (dx == -1) return WEST;
  return dy == 1 ? NORTH : SOUTH;
}
}  // namespace

void Mouse::init() {
  motion_.init();
  if (!hw::load_maze(maze_) || !maze_.sane()) maze_.reset();
  st_ = St::MENU;
  hw::log("micromouse ready, mode %d\r\n", static_cast<int>(mode_));
}

void Mouse::tick(const TickIn& in, TickOut& out) {
  motion_.tick(in, out);
  if (st_ != St::MENU || click_req_ <= 0) return;
  // phan hoi che do bang "tieng click" cua motor: moi click 30 ms toi + 30 ms lui
  click_t_++;
  const int ph = click_phase_;
  if (ph == 0) { out.duty_l = out.duty_r = 0.25f; if (click_t_ >= 30) { click_phase_ = 1; click_t_ = 0; } }
  else if (ph == 1) { out.duty_l = out.duty_r = -0.25f; if (click_t_ >= 30) { click_phase_ = 2; click_t_ = 0; } }
  else if (click_t_ >= 180) { click_phase_ = 0; click_t_ = 0; click_req_ = click_req_ - 1; }
}

void Mouse::start(Mode m) {
  mode_ = m;
  plan_n_ = 0;
  plan_i_ = 0;
  if (m == Mode::AUTO) {
    if (!path_proven()) plan_[plan_n_++] = Mode::EXPLORE;
    plan_[plan_n_++] = Mode::FAST_SAFE;
    plan_[plan_n_++] = Mode::FAST_MAX;
  } else {
    plan_[plan_n_++] = m;
  }
  at_center_ = false;
  runs_done_ = 0;
  for (auto& g : goal_ms_) g = 0;
  st_ = St::ARMED;
  armed_cal_ = false;
  t_state_ = hw::millis();
}

void Mouse::poll_btn(Btn& b, bool raw, uint32_t now) {
  b.pressed = b.released = false;
  if (raw != b.raw) {
    b.raw = raw;
    b.t_raw = now;
  } else if (raw != b.down && now - b.t_raw >= cfg::BTN_STABLE_MS) {
    b.down = raw;
    if (raw) { b.pressed = true; b.t_down = now; }
    else b.released = true;
  }
}

// Reset phan mem: dung motor ngay, giu ban do da do, ve menu cho bam Start lai
void Mouse::abort_run() {
  motion_.abort();
  motion_.clear_fault();
  run_ = Run::NONE;
  finishing_ = false;
  plan_n_ = plan_i_ = 0;
  hw::save_maze(maze_);
  st_ = St::MENU;
  click_phase_ = 0;
  click_t_ = 0;
  click_req_ = 1;
  hw::log("stopped by button\r\n");
}

void Mouse::menu() {
  if (b_mode_.pressed) {
    mode_ = static_cast<Mode>((static_cast<int>(mode_) + 1) % static_cast<int>(Mode::COUNT));
    click_req_ = static_cast<int>(mode_) + 1;
    hw::log("mode %d\r\n", static_cast<int>(mode_));
  }
  // Start tinh luc NHA nut (de phan biet bam nhanh voi giu de reset)
  if (b_start_.released) {
    if (b_start_.skip_release) {
      b_start_.skip_release = false;
    } else if (mode_ == Mode::ERASE) {
      maze_.reset();
      hw::erase_maze();
      click_req_ = 3;
      hw::log("map erased\r\n");
    } else {
      start(mode_);
    }
  }
}

void Mouse::loop() {
  const uint32_t now = hw::millis();
  poll_btn(b_mode_, hw::btn_mode(), now);
  poll_btn(b_start_, hw::btn_start(), now);
  if (b_start_.down && now - b_start_.t_down >= cfg::BTN_RESET_HOLD_MS) {
    abort_run();
    hw::system_reset();
    // chi toi day trong mo phong (tren chip that da khoi dong lai)
    b_start_.skip_release = true;
    b_start_.t_down = now;
    return;
  }
  if (st_ != St::MENU && b_start_.pressed) {
    abort_run();
    b_start_.skip_release = true;
    return;
  }
  if (st_ == St::RUN && motion_.fault()) { handle_fault(); return; }

  switch (st_) {
    case St::MENU:
      menu();
      break;

    case St::ARMED:
      // 1 s cho tay roi khoi robot, roi 0.6 s do bias gyro (robot dung yen)
      if (!armed_cal_) {
        if (now - t_state_ >= 1000) { motion_.start_gyro_cal(600); armed_cal_ = true; }
      } else if (!motion_.gyro_cal_busy()) {
        begin_run(plan_[plan_i_]);
      }
      break;

    case St::RUN:
      if (run_ == Run::EXPLORE) explore_update();
      else feed_moves();
      if (run_ == Run::TEST && mode_ == Mode::TEST_SENSORS) {
        if (now - t_log_ >= 200) {
          t_log_ = now;
          hw::log("IR truoc %d cheoT %d cheoP %d benT %d benP %d" "\r\n", static_cast<int>(motion_.ir(cfg::IR_F)),
                  static_cast<int>(motion_.ir(cfg::IR_DL)), static_cast<int>(motion_.ir(cfg::IR_DR)),
                  static_cast<int>(motion_.ir(cfg::IR_L)), static_cast<int>(motion_.ir(cfg::IR_R)));
        }
        break;  // thoat bang nut SW2 (abort_run)
      }
      if (!motion_.busy() && moves_fed_ >= moves_n_) {
        if (run_ == Run::FAST) {
          goal_ms_[runs_done_] = now - t_run_;
          hw::log("run %d goal %lu ms\r\n", runs_done_ + 1, static_cast<unsigned long>(goal_ms_[runs_done_]));
          hw::save_maze(maze_);
          // dung yen trong dich: do lai bias gyro truoc khi ve
          motion_.enable(false);
          motion_.start_gyro_cal(400);
          run_ = Run::GOAL_CAL;
        } else if (run_ == Run::GOAL_CAL) {
          if (!motion_.gyro_cal_busy()) {
            motion_.enable(true);
            // can goc: tuong truoc + 1 tuong ben -> dat lai ca vi tri ngang lan huong truoc khi ve
            if (cfg::GOAL_CORNER_ALIGN && maze_.wall(cx_, cy_, head_)) motion_.push(mk_front_align());
            for (int k : {-1, 1}) {
              if (!cfg::GOAL_CORNER_ALIGN) break;
              const Dir side = dir_add(head_, k);
              if (!maze_.wall(cx_, cy_, side)) continue;
              motion_.push(mk_pivot(k == -1 ? PI / 2 : -PI / 2));
              motion_.push(mk_front_align());
              head_ = side;
              break;
            }
            // ve nha kieu do duong: cham hon nhung moi o deu sua vi tri theo tuong + kiem tra lai ban do
            if (!cfg::FAST_RETURN || !return_begin()) explore_begin(Phase::HOME, false);
          }
        } else if (run_ == Run::RETURN || run_ == Run::TEST || finishing_) {
          end_run();
        }
      }
      break;

    case St::REST:
      if (now - t_state_ >= cfg::RUN_REST_MS && !motion_.gyro_cal_busy()) {
        if (plan_i_ < plan_n_) {
          begin_run(plan_[plan_i_]);
        } else {
          st_ = St::MENU;
          click_req_ = 2;
          hw::log("session done\r\n");
        }
      }
      break;
  }
}

void Mouse::begin_run(Mode m) {
  hw::log("bat dau luot %d, che do %d\r\n", runs_done_ + 1, static_cast<int>(m));
  plan_i_++;
  finishing_ = false;
  moves_n_ = moves_fed_ = 0;
  cx_ = cy_ = 0;
  head_ = NORTH;
  motion_.clear_fault();
  motion_.enable(true);
  t_run_ = hw::millis();
  st_ = St::RUN;
  switch (m) {
    case Mode::EXPLORE:
      explore_begin(Phase::TO_GOAL, !at_center_);
      break;
    case Mode::FAST_SAFE:
    case Mode::FAST_MAX:
      if (!fast_begin(m == Mode::FAST_MAX ? cfg::RUN_FAST : cfg::RUN_SAFE)) {
        hw::log("no known path -> explore\r\n");
        explore_begin(Phase::TO_GOAL, !at_center_);
      }
      break;
    case Mode::TEST_STRAIGHT: {
      cfg::RunParams rp = cfg::RUN_SAFE;
      rp.v_max = 500;
      rp.accel = rp.decel = 1500;
      moves_[moves_n_++] = mk_straight(5 * cfg::CELL, rp, 0, 0, -1);
      run_ = Run::TEST;
      break;
    }
    case Mode::TEST_PIVOT:
      moves_[moves_n_++] = mk_pivot(10 * 2 * PI);
      run_ = Run::TEST;
      break;
    default:
      motion_.enable(false);
      run_ = Run::TEST;
      break;
  }
}

void Mouse::end_run() {
  motion_.enable(false);
  run_ = Run::NONE;
  runs_done_++;
  hw::save_maze(maze_);
  if (mode_ == Mode::TEST_STRAIGHT || mode_ == Mode::TEST_PIVOT || mode_ == Mode::TEST_SENSORS) {
    st_ = St::MENU;
    return;
  }
  // dung tai o xuat phat >= 2 s roi tu chay luot ke tiep
  at_center_ = true;
  motion_.start_gyro_cal(800);
  st_ = St::REST;
  t_state_ = hw::millis();
}

void Mouse::handle_fault() {
  motion_.abort();
  motion_.clear_fault();
  fault_seen_ = true;
  run_ = Run::NONE;
  hw::save_maze(maze_);
  hw::log("FAULT at cell %d,%d\r\n", cx_, cy_);
  st_ = St::MENU;
  click_req_ = 5;
}

// ---------------------------------------------------------------- do duong

void Mouse::update_wall(int x, int y, Dir d, bool present) {
  if (fixed_wall(x, y, d)) return;
  maze_.set_wall(x, y, d, present);
}

void Mouse::explore_begin(Phase ph, bool from_back) {
  run_ = Run::EXPLORE;
  phase_ = ph;
  finishing_ = false;
  moves_n_ = moves_fed_ = 0;
  if (ph == Phase::HOME && cx_ == 0 && cy_ == 0) { finish_home(); return; }
  // Robot dang dung tai tam o (cx_, cy_) hoac sat tuong sau o xuat phat
  CellMask tgt;
  if (!explore_target(tgt)) { handle_fault(); return; }
  uint16_t dist[cfg::N * cfg::N];
  flood(maze_, tgt, true, dist);
  Dir d = head_;
  if (!explore_step(maze_, dist, cx_, cy_, head_, d)) { handle_fault(); return; }
  if (d != head_) {
    const int k = (d - head_ + 4) & 3;
    motion_.push(mk_pivot(k == 1 ? -PI / 2 : (k == 3 ? PI / 2 : PI)));
    head_ = d;
    from_back = false;
  }
  const float off = from_back ? cfg::START_OFFSET : 0.0f;
  const float len = cfg::HALF - off;
  Move m = mk_straight(len, cfg::RUN_SEARCH, cfg::RUN_SEARCH.v_search, F_SENSE | F_WALL | F_EDGE, len);
  m.sense_at = len - cfg::SENSE_BEFORE_EDGE;
  motion_.push(m);
}

bool Mouse::explore_target(CellMask& tgt) {
  tgt.clear();
  const uint32_t now = hw::millis();
  if (phase_ == Phase::SEARCH) {
    if (now - t_run_ > static_cast<uint32_t>(cfg::EXPLORE_TIME_LIMIT_S * 1000)) {
      phase_ = Phase::HOME;
    } else {
      Route r;
      if (find_route(maze_, START_CELL, NORTH, goal_mask(), true, r)) {
        for (int i = 0; i < r.n; i++) {
          const uint8_t c = r.cell[i];
          if (!maze_.visited(cell_x(c), cell_y(c))) tgt.set(c);
        }
      }
      if (tgt.empty()) {
        hw::log("shortest path proven\r\n");
        phase_ = Phase::HOME;
      }
    }
  }
  if (phase_ == Phase::TO_GOAL) tgt = goal_mask();
  if (phase_ == Phase::IN_GOAL) {
    for (int dy = 0; dy < 2; dy++)
      for (int dx = 0; dx < 2; dx++) {
        const uint8_t c = cell_id(cfg::GOAL_X0 + dx, cfg::GOAL_Y0 + dy);
        if (c != first_goal_) tgt.set(c);
      }
  }
  if (phase_ == Phase::HOME) tgt = single_mask(0, 0);
  return !tgt.empty();
}

void Mouse::explore_update() {
  if (finishing_) return;
  WallObs o;
  if (!motion_.take_obs(o)) return;

  const int nx = cx_ + DX[head_], ny = cy_ + DY[head_];
  if (!in_maze(nx, ny)) { handle_fault(); return; }
  update_wall(nx, ny, head_, o.front);
  update_wall(nx, ny, dir_left(head_), o.left);
  update_wall(nx, ny, dir_right(head_), o.right);
  update_wall(nx, ny, dir_back(head_), false);
  cx_ = nx;
  cy_ = ny;
  const uint8_t c = cell_id(cx_, cy_);
#ifdef MM_DEBUG_LOG
  hw::log("obs cell %d,%d h%d L%d F%d R%d (%d %d %d %d)\n", cx_, cy_, head_, o.left, o.front, o.right,
          static_cast<int>(o.dl), static_cast<int>(o.dfl), static_cast<int>(o.dfr), static_cast<int>(o.dr));
#endif

  if (phase_ == Phase::TO_GOAL && is_goal(c)) {
    goal_ms_[runs_done_] = hw::millis() - t_run_;
    hw::log("explore goal %lu ms\r\n", static_cast<unsigned long>(goal_ms_[runs_done_]));
    first_goal_ = c;
    phase_ = Phase::IN_GOAL;
  } else if (phase_ == Phase::IN_GOAL && is_goal(c) && c != first_goal_) {
    phase_ = Phase::SEARCH;
  }

  if (phase_ == Phase::HOME && cx_ == 0 && cy_ == 0) { finish_home(); return; }

  // attempt 0: binh thuong; 1: SEARCH -> HOME; 2: quen tuong quanh robot; 3: xoa ban do, do lai
  for (int attempt = 0; attempt < 4; attempt++) {
    if (attempt == 2) {
      hw::log("map conflict at %d,%d: forget nearby walls\r\n", cx_, cy_);
      for (int y = cy_ - 3; y <= cy_ + 3; y++)
        for (int x = cx_ - 3; x <= cx_ + 3; x++)
          if (in_maze(x, y))
            for (int d = 0; d < 4; d++)
              if (!fixed_wall(x, y, static_cast<Dir>(d)) && !(x == cx_ && y == cy_) &&
                  !(x + DX[d] == cx_ && y + DY[d] == cy_))  // giu tuong o hien tai: vua do xong
                maze_.forget(x, y, static_cast<Dir>(d));
    } else if (attempt == 3) {
      hw::log("map conflict: reset map\r\n");
      const uint8_t keep = maze_.raw(cell_id(cx_, cy_));
      maze_.reset();
      for (int d = 0; d < 4; d++)
        if (keep & (16u << d)) update_wall(cx_, cy_, static_cast<Dir>(d), keep & (1u << d));
    }
    CellMask tgt;
    if (!explore_target(tgt)) continue;
    if (phase_ == Phase::HOME && cx_ == 0 && cy_ == 0) { finish_home(); return; }
    uint16_t dist[cfg::N * cfg::N];
    flood(maze_, tgt, true, dist);
    Dir d;
    if (explore_step(maze_, dist, cx_, cy_, head_, d)) { explore_push(d); return; }
    if (phase_ == Phase::SEARCH) phase_ = Phase::HOME;
  }
  handle_fault();
}

void Mouse::explore_push(Dir d) {
  const cfg::RunParams& rp = cfg::RUN_SEARCH;
  const int rel = (d - head_ + 4) & 3;
  if (rel == 0) {
    Move m = mk_straight(cfg::CELL, rp, rp.v_search, F_SENSE | F_WALL | F_EDGE, 0.0f);
    m.sense_at = cfg::CELL - cfg::SENSE_BEFORE_EDGE;
    motion_.push(m);
  } else if (rel == 1 || rel == 3) {
    Move m = mk_turn(T_SEARCH90, rel == 3 ? 1 : -1, rp);
    m.flags = F_SENSE;
    m.sense_at = m.dist - cfg::SENSE_BEFORE_EDGE;
    motion_.push(m);
  } else {
    motion_.push(mk_straight(cfg::HALF, rp, 0, F_WALL, -1));
    if (maze_.wall(cx_, cy_, head_)) motion_.push(mk_front_align());
    motion_.push(mk_pivot(PI));
    Move m = mk_straight(cfg::HALF, rp, rp.v_search, F_SENSE | F_WALL | F_EDGE, cfg::HALF);
    m.sense_at = cfg::HALF - cfg::SENSE_BEFORE_EDGE;
    motion_.push(m);
  }
  head_ = d;
}

void Mouse::finish_home() {
  // dang o mep o xuat phat, huong NAM: vao tam, can tuong, quay mat ra
  motion_.push(mk_straight(cfg::HALF, cfg::RUN_SEARCH, 0, F_WALL, -1));
  // can tuong nam, quay phai can tuong tay, quay phai -> huong bac, dung tam o
  motion_.push(mk_front_align());
  if (cfg::START_CORNER_ALIGN) {
    motion_.push(mk_pivot(-PI / 2));
    motion_.push(mk_front_align());
    motion_.push(mk_pivot(-PI / 2));
  } else {
    motion_.push(mk_pivot(PI));
  }
  head_ = NORTH;
  finishing_ = true;
  run_ = Run::EXPLORE;
  moves_n_ = moves_fed_ = 0;
}

// ---------------------------------------------------------------- chay nhanh

bool Mouse::build_moves(const Route& r, Dir heading, float offset, const cfg::RunParams& rp,
                        uint8_t end_cell, Dir end_head) {
  // thu lan luot: cheo + cua lon -> chi cua lon -> chi cua nho (luon chay duoc)
  for (int level = 0; level < 3; level++) {
    cfg::RunParams p = rp;
    if (level >= 1) p.diagonal = false;
    if (level >= 2) p.big_turns = false;
    const int n = compile_route(r, heading, offset, p, moves_, MAXM);
    if (n <= 0 || !verify_moves(moves_, n, r.cell[0], heading, offset, end_cell, end_head)) continue;
    const float cl = path_clearance(moves_, n, r.cell[0], heading, offset, maze_);
    hw::log("plan level %d: %d moves, clearance %d mm\r\n", level, n, static_cast<int>(cl));
    if (cl >= cfg::CLEAR_MIN_MM || level == 2) {
      moves_n_ = n;
      moves_fed_ = 0;
      return true;
    }
  }
  return false;
}

bool Mouse::fast_begin(const cfg::RunParams& rp) {
  Route r;
  if (!find_route(maze_, START_CELL, NORTH, goal_mask(), false, r) || r.n < 2) return false;
  // di them 1 o thang trong vung dich de ca robot nam tron trong dich khi dung
  const Dir h = dir_between(r.cell[r.n - 2], r.cell[r.n - 1]);
  const uint8_t last = r.cell[r.n - 1];
  const int ex = cell_x(last) + DX[h], ey = cell_y(last) + DY[h];
  if (!in_maze(ex, ey) || !is_goal(ex, ey) || maze_.wall(cell_x(last), cell_y(last), h)) return false;
  r.cell[r.n] = cell_id(ex, ey);
  r.act[r.n] = ACT_S;
  r.n++;
  const float off = at_center_ ? 0.0f : cfg::START_OFFSET;
  if (!build_moves(r, NORTH, off, rp, cell_id(ex, ey), h)) return false;
  cx_ = ex;
  cy_ = ey;
  head_ = h;
  run_ = Run::FAST;
  t_run_ = hw::millis();
  feed_moves();
  return true;
}

bool Mouse::return_begin() {
  Route r;
  if (!find_route(maze_, cell_id(cx_, cy_), head_, single_mask(0, 0), false, r) || r.n < 2) return false;
  const Dir h = dir_between(r.cell[r.n - 2], r.cell[r.n - 1]);
  if (!build_moves(r, head_, 0.0f, cfg::RUN_RETURN, START_CELL, h)) return false;
  if (moves_n_ + 2 > MAXM) return false;
  moves_[moves_n_++] = mk_front_align();
  moves_[moves_n_++] = mk_pivot(PI);
  cx_ = cy_ = 0;
  head_ = NORTH;
  run_ = Run::RETURN;
  feed_moves();
  return true;
}

void Mouse::feed_moves() {
  while (moves_fed_ < moves_n_ && motion_.free_slots() > 0) motion_.push(moves_[moves_fed_++]);
}

bool Mouse::path_proven() {
  Route known, opt;
  if (!find_route(maze_, START_CELL, NORTH, goal_mask(), false, known)) return false;
  if (!find_route(maze_, START_CELL, NORTH, goal_mask(), true, opt)) return false;
  return known.cost == opt.cost;
}

}  // namespace mm

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
}

void Mouse::init() {
  motion_.init();
  if (!hw::load_maze(maze_) || !maze_.sane()) maze_.reset();
  st_ = St::MENU;
  hw::log("micromouse ready, mode %d\r\n", static_cast<int>(mode_));
}

void Mouse::tick(const TickIn& in, TickOut& out) {
  motion_.tick(in, out);
  if (st_ != St::MENU || click_req_ <= 0) return;
  click_t_++;
  if (click_phase_ == 0) {
    const float d = (click_t_ & 1u) ? 0.18f : -0.18f;
    out.duty_l = out.duty_r = d;
    if (click_t_ >= 120) { click_phase_ = 1; click_t_ = 0; }
  } else if (click_t_ >= 180) {
    click_phase_ = 0;
    click_t_ = 0;
    click_req_ = click_req_ - 1;
  }
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

void Mouse::go() {
  if (mode_ == Mode::ERASE) {
    maze_.reset();
    hw::erase_maze();
    click_req_ = 3;
    hw::log("map erased\r\n");
  } else {
    start(mode_);
  }
}

void Mouse::menu() {
  const uint32_t now = hw::millis();
  if (b_mode_.down && !b_mode_.long_fb && !b_mode_.skip_release && now - b_mode_.t_down >= cfg::BTN_LONG_MS) {
    b_mode_.long_fb = true;
    click_phase_ = 0;
    click_t_ = 0;
    click_req_ = 1;
  }
  if (b_mode_.released) {
    const bool was_long = b_mode_.long_fb;
    b_mode_.long_fb = false;
    if (b_mode_.skip_release) {
      b_mode_.skip_release = false;
    } else if (was_long) {
      go();
    } else if (cfg::LOCK_FOLLOW) {
      mode_ = Mode::FOLLOW;
      click_phase_ = 0;
      click_t_ = 0;
      click_req_ = 2;
      hw::log("bam nhanh SW3: dang khoa che do 1 (giu SW3 ~1 s de chay)\r\n");
    } else {
      mode_ = static_cast<Mode>((static_cast<int>(mode_) + 1) % static_cast<int>(Mode::COUNT));
      click_phase_ = 0;
      click_t_ = 0;
      click_req_ = static_cast<int>(mode_) + 1;
      hw::log("mode %d\r\n", static_cast<int>(mode_));
    }
  }
  if (b_start_.released) {
    if (b_start_.skip_release) b_start_.skip_release = false;
    else go();
  }
}

void Mouse::loop() {
  const uint32_t now = hw::millis();
  poll_btn(b_mode_, hw::btn_mode(), now);
  poll_btn(b_start_, hw::btn_start(), now);
  if (b_start_.down && now - b_start_.t_down >= cfg::BTN_RESET_HOLD_MS) {
    abort_run();
    hw::system_reset();
    b_start_.skip_release = true;
    b_start_.t_down = now;
    return;
  }
  if (st_ != St::MENU && (b_start_.pressed || b_mode_.pressed)) {
    abort_run();
    b_start_.skip_release = b_start_.down;
    b_mode_.skip_release = b_mode_.down;
    return;
  }
  if (st_ == St::RUN && motion_.fault()) { handle_fault(); return; }

  switch (st_) {
    case St::MENU:
      menu();
      break;

    case St::ARMED:
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
        break;
      }
      if (!motion_.busy() && moves_fed_ >= moves_n_) {
        if (run_ == Run::FAST) {
          goal_ms_[runs_done_] = now - t_run_;
          hw::log("run %d goal %lu ms\r\n", runs_done_ + 1, static_cast<unsigned long>(goal_ms_[runs_done_]));
          hw::save_maze(maze_);
          motion_.enable(false);
          motion_.start_gyro_cal(400);
          run_ = Run::GOAL_CAL;
        } else if (run_ == Run::GOAL_CAL) {
          if (!motion_.gyro_cal_busy()) {
            motion_.enable(true);
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
  {
    float gmm;
    int gturn;
    motion_.take_guard(gmm, gturn);
  }
  plan_i_++;
  finishing_ = false;
  moves_n_ = moves_fed_ = 0;
  cx_ = cy_ = 0;
  head_ = NORTH;
  motion_.clear_fault();
  motion_.enable(true);
  t_run_ = hw::millis();
  st_ = St::RUN;
  follow_ = m == Mode::FOLLOW;
  look_ = Look::NONE;
  look_retry_ = false;
  switch (m) {
    case Mode::FOLLOW:
      hw::log("FOLLOW: di thang, gap tuong truoc re ve phia trong\r\n");
      explore_begin(Phase::TO_GOAL, !at_center_);
      break;
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
  Dir d = head_;
  if (!follow_) {
    CellMask tgt;
    if (!explore_target(tgt)) { handle_fault(); return; }
    uint16_t dist[cfg::N * cfg::N];
    flood(maze_, tgt, true, dist);
    if (!explore_step(maze_, dist, cx_, cy_, head_, d)) {
      hw::log("ban do cu khong con duong toi dich -> xoa ban do, do lai\r\n");
      maze_.reset();
      hw::erase_maze();
      flood(maze_, tgt, true, dist);
      d = head_;
      if (!explore_step(maze_, dist, cx_, cy_, head_, d)) { handle_fault(); return; }
    }
  }
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
  if (follow_ || cfg::EXPLORE_LOOK) {
    float gmm = 0;
    int gturn = 0;
    if (motion_.take_guard(gmm, gturn)) {
      if (in_maze(cx_, cy_)) update_wall(cx_, cy_, head_, true);
      if (gturn != 0) {
        head_ = gturn > 0 ? dir_left(head_) : dir_right(head_);
        hw::log("FOLLOW (%d,%d): thay tuong truoc (%d mm) -> cua cong %s\r\n", cx_, cy_, static_cast<int>(gmm),
                gturn > 0 ? "trai" : "phai");
      } else if (look_ == Look::NONE) {
        hw::log("(%d,%d): thay tuong truoc (%d mm) -> phanh, dung nhin\r\n", cx_, cy_, static_cast<int>(gmm));
        look_ = Look::WAIT_STOP;
      }
    }
    if (look_ != Look::NONE) {
      follow_look();
      return;
    }
  }
  WallObs o;
  if (!motion_.take_obs(o)) return;

  const int nx = cx_ + DX[head_], ny = cy_ + DY[head_];
  const bool inside = in_maze(nx, ny);
  if (!inside && !follow_) { handle_fault(); return; }
  const bool look_here = cfg::EXPLORE_LOOK && !follow_ && inside && !cell_known(nx, ny);
  if (inside) {
    if (cfg::EXPLORE_LOOK && !follow_) {
      update_wall(nx, ny, dir_back(head_), false);
    } else {
      update_wall(nx, ny, head_, o.front);
      update_wall(nx, ny, dir_left(head_), o.left);
      update_wall(nx, ny, dir_right(head_), o.right);
      update_wall(nx, ny, dir_back(head_), false);
    }
  }
  cx_ = nx;
  cy_ = ny;
  obs_front_ = o.front;
  const uint8_t c = inside ? cell_id(cx_, cy_) : 0;
#ifdef MM_DEBUG_LOG
  hw::log("obs cell %d,%d h%d L%d F%d R%d (%d %d %d %d)\n", cx_, cy_, head_, o.left, o.front, o.right,
          static_cast<int>(o.dl), static_cast<int>(o.dfl), static_cast<int>(o.dfr), static_cast<int>(o.dr));
#endif

  if (follow_) {
    if ((inside && is_goal(c)) || hw::millis() - t_run_ > 120000u) {
      motion_.push(mk_straight(cfg::HALF, cfg::RUN_SEARCH, 0, F_WALL, -1));
      finishing_ = true;
      hw::log("FOLLOW: dung o o %d,%d (%s)\r\n", cx_, cy_, is_goal(c) ? "toi dich" : "het 2 phut");
      return;
    }
    const int8_t turn_dir = !o.left ? 1 : (!o.right ? -1 : 0);
    const bool maybe = o.fpct2 >= cfg::FRONT_MAYBE_PCT && (!cfg::FOLLOW_CURVE || o.fmin < cfg::DYN_EDGE_STOP_MM);
    hw::log("FOLLOW (%d,%d) T%d[%d-%d %d%% %d%%] L%d %d R%d %d -> %s\r\n", cx_, cy_, o.front,
            static_cast<int>(o.fmin), static_cast<int>(o.fmax), o.fpct, o.fpct2, o.left, static_cast<int>(o.dl),
            o.right, static_cast<int>(o.dr), maybe ? "tuong gan, dung nhin" : "thang");
    if (maybe) {
      const float len = std::fmax(10.0f, std::fmin(cfg::HALF, o.fmin - cfg::LOOK_STOP_MM - cfg::SENSE_BEFORE_EDGE));
      motion_.push(mk_straight(len, cfg::RUN_SEARCH, 0, F_WALL, -1));
      look_ = Look::WAIT_STOP;
      return;
    }
    const cfg::RunParams& rp = cfg::RUN_SEARCH;
    Move m = mk_straight(cfg::CELL, rp, rp.v_search, F_SENSE | F_WALL | F_EDGE | F_FGUARD, 0.0f);
    m.sense_at = cfg::CELL - cfg::SENSE_BEFORE_EDGE;
    m.dir = turn_dir;
    motion_.push(m);
    return;
  }

  if (look_here) {
    motion_.push(mk_straight(cfg::HALF, cfg::RUN_SEARCH, 0, F_WALL | F_FGUARD, -1));
    look_ = Look::WAIT_STOP;
    look_adv_ = 0.0f;
    return;
  }
  Dir d;
  if (!explore_decide(d, false, 0.0f)) return;
  if (cfg::EXPLORE_LOOK && !follow_ && d != head_) {
    motion_.push(mk_straight(cfg::HALF, cfg::RUN_SEARCH, 0, F_WALL | F_FGUARD, -1));
    look_ = Look::WAIT_STOP;
    look_adv_ = 0.0f;
    return;
  }
  explore_push(d);
}

bool Mouse::cell_known(int x, int y) const {
  for (int k = 0; k < 4; k++)
    if (!maze_.known(x, y, static_cast<Dir>(k))) return false;
  return true;
}

bool Mouse::explore_decide(Dir& d, bool at_center, float creep) {
  const uint8_t c = cell_id(cx_, cy_);
  if (phase_ == Phase::TO_GOAL && is_goal(c)) {
    goal_ms_[runs_done_] = hw::millis() - t_run_;
    hw::log("explore goal %lu ms\r\n", static_cast<unsigned long>(goal_ms_[runs_done_]));
    first_goal_ = c;
    phase_ = Phase::IN_GOAL;
  } else if (phase_ == Phase::IN_GOAL && is_goal(c) && c != first_goal_) {
    phase_ = Phase::SEARCH;
  }

  if (phase_ == Phase::HOME && cx_ == 0 && cy_ == 0) { finish_home(at_center, creep); return false; }

  for (int attempt = 0; attempt < 4; attempt++) {
    if (attempt == 2) {
      hw::log("map conflict at %d,%d: forget nearby walls\r\n", cx_, cy_);
      for (int y = cy_ - 3; y <= cy_ + 3; y++)
        for (int x = cx_ - 3; x <= cx_ + 3; x++)
          if (in_maze(x, y))
            for (int d = 0; d < 4; d++)
              if (!fixed_wall(x, y, static_cast<Dir>(d)) && !(x == cx_ && y == cy_) &&
                  !(x + DX[d] == cx_ && y + DY[d] == cy_))
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
    if (phase_ == Phase::HOME && cx_ == 0 && cy_ == 0) { finish_home(at_center, creep); return false; }
    uint16_t dist[cfg::N * cfg::N];
    flood(maze_, tgt, true, dist);
    if (explore_step(maze_, dist, cx_, cy_, head_, d)) return true;
    if (phase_ == Phase::SEARCH) phase_ = Phase::HOME;
  }
  handle_fault();
  return false;
}

void Mouse::follow_look() {
  const uint32_t now = hw::millis();
  if (look_ == Look::WAIT_STOP) {
    if (motion_.busy()) return;
    look_ = Look::SAMPLE;
    look_t_ = look_ms_ = now;
    look_n_ = look_f_ = look_l_ = look_r_ = 0;
    for (float& v : look_sum_) v = 0;
    return;
  }
  if (now == look_ms_) return;
  look_ms_ = now;
  if (now - look_t_ < cfg::LOOK_WAIT_MS) return;
  look_n_++;
  for (int i = 0; i < cfg::IR_COUNT; i++) look_sum_[i] += motion_.ir(i);
  if (motion_.ir(cfg::IR_F) < cfg::LOOK_FRONT_TH) look_f_++;
  if (motion_.ir(cfg::IR_L) < cfg::SIDE_WALL_TH) look_l_++;
  if (motion_.ir(cfg::IR_R) < cfg::SIDE_WALL_TH) look_r_++;
  if (now - look_t_ < cfg::LOOK_WAIT_MS + cfg::LOOK_MS) return;

  const bool front = look_f_ * 2 > look_n_, left = look_l_ * 2 > look_n_, right = look_r_ * 2 > look_n_;
  const float fav = look_sum_[cfg::IR_F] / static_cast<float>(look_n_);
  if (!front && fav < cfg::LOOK_AMBIG_MM && !look_retry_) {
    hw::log("  dung nhin (%d,%d): truoc %d mm khong chac -> bo them %d mm nhin lai\r\n", cx_, cy_,
            static_cast<int>(fav), static_cast<int>(cfg::LOOK_RETRY_MM));
    cfg::RunParams rc = cfg::RUN_SEARCH;
    rc.v_max = cfg::CREEP_V;
    rc.accel = rc.decel = 1000.0f;
    motion_.push(mk_straight(cfg::LOOK_RETRY_MM, rc, 0, 0, -1));
    look_adv_ += cfg::LOOK_RETRY_MM;
    look_retry_ = true;
    look_ = Look::WAIT_STOP;
    return;
  }
  look_retry_ = false;
  if (in_maze(cx_, cy_)) {
    update_wall(cx_, cy_, head_, front);
    update_wall(cx_, cy_, dir_left(head_), left);
    update_wall(cx_, cy_, dir_right(head_), right);
  }
  const auto avg = [&](int i) { return static_cast<int>(look_sum_[i] / static_cast<float>(look_n_)); };
  const float creep = front ? static_cast<float>(avg(cfg::IR_F)) - cfg::CREEP_TARGET_MM : 0.0f;
  if (!follow_) {
    look_ = Look::NONE;
    hw::log("  nhin (%d,%d): T%d %d L%d %d R%d %d\r\n", cx_, cy_, front, avg(cfg::IR_F), left, avg(cfg::IR_L), right,
            avg(cfg::IR_R));
    Dir dn;
    if (explore_decide(dn, true, creep)) center_push(dn, front, creep);
    return;
  }
  const Dir l = dir_left(head_), r = dir_right(head_);
  Dir d = head_;
  if (front) d = !left ? l : (!right ? r : dir_back(head_));
  hw::log("  dung nhin (%d,%d): T%d %d L%d %d R%d %d cheo %d %d -> %s\r\n", cx_, cy_, front, avg(cfg::IR_F), left,
          avg(cfg::IR_L), right, avg(cfg::IR_R), avg(cfg::IR_DL), avg(cfg::IR_DR),
          d == head_ ? "khong co tuong, di tiep" : (d == l ? "re trai" : (d == r ? "re phai" : "quay dau")));
  look_ = Look::NONE;
  center_push(d, front, creep);
}

void Mouse::center_push(Dir d, bool front, float creep) {
  const cfg::RunParams& rp = cfg::RUN_SEARCH;
  if (front) {
    if (creep > 3.0f) {
      cfg::RunParams rc = rp;
      rc.v_max = cfg::CREEP_V;
      rc.accel = rc.decel = 1000.0f;
      motion_.push(mk_straight(creep, rc, 0, 0, -1));
    }
    motion_.push(mk_front_align());
  }
  const int rel = (d - head_ + 4) & 3;
  if (rel != 0) motion_.push(mk_pivot(rel == 1 ? -PI / 2 : (rel == 3 ? PI / 2 : PI)));
  const float len = (rel == 0 && !front) ? std::fmax(20.0f, cfg::HALF - look_adv_) : cfg::HALF;
  Move m = mk_straight(len, rp, rp.v_search, F_SENSE | F_WALL | F_EDGE, len);
  m.sense_at = len - cfg::SENSE_BEFORE_EDGE;
  motion_.push(m);
  head_ = d;
  obs_front_ = front;
  look_adv_ = 0.0f;
}

void Mouse::explore_push(Dir d) {
  const cfg::RunParams& rp = cfg::RUN_SEARCH;
  const int rel = (d - head_ + 4) & 3;
  if (rel == 0) {
    const uint8_t guard = (cfg::EXPLORE_LOOK && !follow_) ? F_FGUARD : 0;
    Move m = mk_straight(cfg::CELL, rp, rp.v_search, F_SENSE | F_WALL | F_EDGE | guard, 0.0f);
    m.sense_at = cfg::CELL - cfg::SENSE_BEFORE_EDGE;
    motion_.push(m);
  } else if (rel == 1 || rel == 3) {
    Move m = mk_turn(T_SEARCH90, rel == 3 ? 1 : -1, rp);
    m.flags = F_SENSE;
    m.sense_at = m.dist - cfg::SENSE_BEFORE_EDGE;
    motion_.push(m);
  } else {
    motion_.push(mk_straight(cfg::HALF, rp, 0, F_WALL, -1));
    const bool front_wall = (follow_ || !in_maze(cx_, cy_)) ? obs_front_ : maze_.wall(cx_, cy_, head_);
    if (front_wall) motion_.push(mk_front_align());
    motion_.push(mk_pivot(PI));
    Move m = mk_straight(cfg::HALF, rp, rp.v_search, F_SENSE | F_WALL | F_EDGE, cfg::HALF);
    m.sense_at = cfg::HALF - cfg::SENSE_BEFORE_EDGE;
    motion_.push(m);
  }
  head_ = d;
}

void Mouse::finish_home(bool at_center, float creep) {
  if (!at_center) {
    motion_.push(mk_straight(cfg::HALF, cfg::RUN_SEARCH, 0, F_WALL, -1));
  } else if (creep > 3.0f) {
    cfg::RunParams rc = cfg::RUN_SEARCH;
    rc.v_max = cfg::CREEP_V;
    rc.accel = rc.decel = 1000.0f;
    motion_.push(mk_straight(creep, rc, 0, 0, -1));
  }
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

bool Mouse::build_moves(const Route& r, Dir heading, float offset, const cfg::RunParams& rp,
                        uint8_t end_cell, Dir end_head) {
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

}

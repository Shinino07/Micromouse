#include "motion.h"
#include <atomic>
#include <cmath>

namespace mm {

namespace {
constexpr float DT = cfg::DT_S;
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float sgnf(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }
constexpr float V_CREEP = 20.0f;
constexpr float V_TURN_MIN = 80.0f;
constexpr float UNDERRUN_DECEL = 3000.0f;
}

void Motion::init() {
  head_ = tail_ = 0;
  active_ = false;
  enc_init_ = false;
  s_ref_ = s_meas_ = v_ref_ = a_ref_ = 0;
  th_ref_ = th_meas_ = w_ref_ = al_ref_ = 0;
  ms_ = 0;
  for (int i = 0; i < cfg::IR_COUNT; i++) ir_f_[i] = ir_raw_[i] = ir_prev_[i] = cfg::IR_NONE;
  init_turns();
}

void Motion::enable(bool on) {
  if (on && !enabled_) {
    s_ref_ = s_meas_;
    th_ref_ = th_meas_;
    v_ref_ = a_ref_ = w_ref_ = al_ref_ = 0;
    ms_ = 0;
    fault_cnt_ = 0;
    i_pos_ = i_ang_ = 0;
  }
  enabled_ = on;
}

bool Motion::push(const Move& m) {
  const uint16_t h = head_;
  const uint16_t nh = static_cast<uint16_t>((h + 1) % QN);
  if (nh == tail_) return false;
  q_[h] = m;
  std::atomic_signal_fence(std::memory_order_release);
  head_ = nh;
  return true;
}

int Motion::free_slots() const {
  const int used = (head_ - tail_ + QN) % QN;
  return QN - 1 - used;
}

void Motion::abort() {
  enabled_ = false;
  tail_ = head_;
  active_ = false;
}

bool Motion::take_obs(WallObs& o) {
  if (!obs_ready_) return false;
  o = obs_;
  obs_ready_ = false;
  return true;
}

void Motion::start_gyro_cal(uint16_t ms) {
  cal_sum_ = 0;
  cal_n_ = 0;
  cal_left_ = ms;
}

void Motion::start_next() {
  if (head_ == tail_) return;
  std::atomic_signal_fence(std::memory_order_acquire);
  const float carry = ms_;
  mv_ = q_[tail_];
  tail_ = static_cast<uint16_t>((tail_ + 1) % QN);
  active_ = true;
  ms_ = (mv_.type == MoveType::STRAIGHT || mv_.type == MoveType::DIAG || mv_.type == MoveType::TURN) ? carry : 0.0f;
  t_move_ = 0;
  sensed_ = false;
  pre_valid_ = false;
  fw_n_ = fw_hit_ = fw_hit2_ = 0;
  guard_n_ = 0;
  fw_min_ = 300.0f;
  fw_max_ = 0.0f;
  settling_ = false;
  th_start_ = th_ref_;
  phi_ = w_piv_ = 0;
  last_edge_k_[0] = last_edge_k_[1] = -9;
  last_rise_k_[0] = last_rise_k_[1] = -9;
  if (mv_.type == MoveType::DIAG) {
    dmin_[0] = dmin_[1] = cfg::IR_NONE;
    dvalid_[0] = dvalid_[1] = false;
    darmed_[0] = darmed_[1] = false;
    lat_valid_ = false;
    lat_est_ = 0;
  }
  diag_active_ = false;
  side_present_[0] = ir_f_[cfg::IR_L] < cfg::SIDE_WALL_TH;
  side_present_[1] = ir_f_[cfg::IR_R] < cfg::SIDE_WALL_TH;
  present_since_[0] = present_since_[1] = ms_ - 1000.0f;
  if (mv_.type == MoveType::TURN) mv_.v_max = std::fmax(V_TURN_MIN, std::fmin(mv_.v_max, std::fmax(v_ref_, V_TURN_MIN)));
  if (mv_.type == MoveType::FRONT_ALIGN) {
    const bool wall = ir_f_[cfg::IR_DL] < 1.6f * cfg::DIAG_FRONT_NOM && ir_f_[cfg::IR_DR] < 1.6f * cfg::DIAG_FRONT_NOM;
    front_align_ = wall;
    if (!wall) active_ = false;
  }
}

bool Motion::front_guard() {
  if (ms_ < cfg::GUARD_S0 || ms_ > cfg::GUARD_S1) {
    guard_n_ = 0;
    return false;
  }
  if (ir_raw_[cfg::IR_F] >= cfg::GUARD_MM) {
    guard_n_ = 0;
    return false;
  }
  if (++guard_n_ < 9) return false;
  const bool open_l = ir_f_[cfg::IR_L] > cfg::SIDE_WALL_TH && ir_raw_[cfg::IR_L] > cfg::SIDE_WALL_TH;
  const bool open_r = ir_f_[cfg::IR_R] > cfg::SIDE_WALL_TH && ir_raw_[cfg::IR_R] > cfg::SIDE_WALL_TH;
  mv_.dir = !cfg::FOLLOW_CURVE ? 0 : (open_l ? 1 : (open_r ? -1 : 0));
  const float axle_wall = ir_raw_[cfg::IR_F] + cfg::FRONT_MOVE_BIAS + cfg::IR_POSE[cfg::IR_F].fwd;
  const float fwd_raw = axle_wall - (cfg::HALF - cfg::WALL_HALF);
  if (cfg::FOLLOW_CURVE && mv_.dir == 0 && fwd_raw > cfg::DYN_FWD_MIN + cfg::DYN_WAIT_MARGIN) return false;
  if (mv_.dir != 0) {
    const float fwd = clampf(fwd_raw, cfg::DYN_FWD_MIN, cfg::DYN_FWD_MAX);
    set_dyn_turn(fwd, cfg::HALF, 1.5707963f);
    const TurnGeom& g = g_turn[T_DYN];
    const int8_t dir = mv_.dir;
    float v = std::sqrt(cfg::RUN_SEARCH.a_lat / g.kmax);
    v = std::fmin(v, 0.9f * cfg::V_WHEEL_MAX / (1.0f + g.kmax * 0.5f * cfg::TRACK_MM));
    v = std::fmax(V_TURN_MIN, std::fmin(v, std::fmax(v_ref_, V_TURN_MIN)));
    mv_.type = MoveType::TURN;
    mv_.turn = T_DYN;
    mv_.dir = dir;
    mv_.dist = g.pre + g.curve + g.post;
    mv_.v_max = mv_.v_end = v;
    mv_.flags = F_SENSE;
    mv_.sense_at = mv_.dist - cfg::SENSE_BEFORE_EDGE;
    mv_.edge_phase = -1.0f;
    ms_ = 0.0f;
    t_move_ = 0.0f;
    th_start_ = th_ref_;
    sensed_ = false;
    pre_valid_ = false;
    fw_n_ = fw_hit_ = fw_hit2_ = 0;
    fw_min_ = 300.0f;
    fw_max_ = 0.0f;
    wall_active_ = false;
    guard_mm_ = ir_raw_[cfg::IR_F];
    guard_turn_ = dir;
    guard_hit_ = true;
    return true;
  }
  mv_.dist = std::fmin(cfg::HALF + 30.0f, ms_ + std::fmax(20.0f, ir_raw_[cfg::IR_F] - cfg::LOOK_STOP_MM));
  mv_.v_end = 0.0f;
  mv_.flags = static_cast<uint8_t>(mv_.flags & ~(F_SENSE | F_FGUARD));
  guard_mm_ = ir_raw_[cfg::IR_F];
  guard_turn_ = 0;
  guard_hit_ = true;
  return false;
}

void Motion::presample() {
  if (!(mv_.flags & F_SENSE) || sensed_) return;
  if (ms_ >= mv_.sense_at - cfg::SENSE_GAP) {
    const float f = ir_raw_[cfg::IR_F];
    fw_n_++;
    if (f < cfg::FRONT_WALL_TH) fw_hit_++;
    if (f < cfg::FRONT_MAYBE_TH) fw_hit2_++;
    fw_min_ = std::fmin(fw_min_, f);
    fw_max_ = std::fmax(fw_max_, f);
  }
  if (pre_valid_ || mv_.sense_at < cfg::SENSE_GAP) return;
  if (ms_ < mv_.sense_at - cfg::SENSE_GAP) return;
  pre_l_ = ir_f_[cfg::IR_L] < cfg::SIDE_WALL_TH;
  pre_r_ = ir_f_[cfg::IR_R] < cfg::SIDE_WALL_TH;
  pre_valid_ = true;
}

void Motion::sense() {
  obs_.dl = ir_f_[cfg::IR_L];
  obs_.dr = ir_f_[cfg::IR_R];
  obs_.dfl = ir_f_[cfg::IR_F];
  obs_.dfr = std::fmin(ir_f_[cfg::IR_DL], ir_f_[cfg::IR_DR]);
  obs_.left = obs_.dl < cfg::SIDE_WALL_TH && (!pre_valid_ || pre_l_);
  obs_.right = obs_.dr < cfg::SIDE_WALL_TH && (!pre_valid_ || pre_r_);
  obs_.front = fw_n_ >= 6 ? (fw_hit_ * 4 >= fw_n_ * 3) : (ir_f_[cfg::IR_F] < cfg::FRONT_WALL_TH);
  obs_.fmin = fw_n_ ? fw_min_ : ir_f_[cfg::IR_F];
  obs_.fmax = fw_n_ ? fw_max_ : ir_f_[cfg::IR_F];
  obs_.fpct = fw_n_ ? fw_hit_ * 100 / fw_n_ : -1;
  obs_.fpct2 = fw_n_ ? fw_hit2_ * 100 / fw_n_ : (ir_f_[cfg::IR_F] < cfg::FRONT_MAYBE_TH ? 100 : 0);
  obs_ready_ = true;
  sensed_ = true;
}

float Motion::wall_error(bool& valid) {
  const float dl = ir_sf_[0], dr = ir_sf_[1];
  const bool ok_l = dl < cfg::SIDE_WALL_TH && ir_raw_[cfg::IR_L] >= cfg::IR_SAT_MM[cfg::IR_L] && s_meas_ >= blank_until_[0];
  const bool ok_r = dr < cfg::SIDE_WALL_TH && ir_raw_[cfg::IR_R] >= cfg::IR_SAT_MM[cfg::IR_R] && s_meas_ >= blank_until_[1];
  valid = ok_l || ok_r;
  if (ok_l && ok_r) return 0.5f * ((dl - cfg::SIDE_NOM_L) - (dr - cfg::SIDE_NOM_R));
  if (ok_l) return dl - cfg::SIDE_NOM_L;
  if (ok_r) return cfg::SIDE_NOM_R - dr;
  if (ir_raw_[cfg::IR_L] < cfg::IR_SAT_MM[cfg::IR_L] && s_meas_ >= blank_until_[0]) {
    valid = true;
    return cfg::IR_SAT_MM[cfg::IR_L] - cfg::SIDE_NOM_L;
  }
  if (ir_raw_[cfg::IR_R] < cfg::IR_SAT_MM[cfg::IR_R] && s_meas_ >= blank_until_[1]) {
    valid = true;
    return cfg::SIDE_NOM_R - cfg::IR_SAT_MM[cfg::IR_R];
  }
  return 0.0f;
}

void Motion::edge_correction() {
  if (!cfg::EDGE_CORR) return;
  if (mv_.edge_phase < 0 || std::fabs(th_ref_ - th_meas_) > 0.1f) return;
  const int ids[2] = {cfg::IR_L, cfg::IR_R};
  for (int i = 0; i < 2; i++) {
    const float d = ir_f_[ids[i]];
    const bool was = side_present_[i];
    if (was && d > cfg::SIDE_WALL_TH + cfg::SIDE_EDGE_HYST) side_present_[i] = false;
    else if (!was && d < cfg::SIDE_WALL_TH) { side_present_[i] = true; present_since_[i] = ms_; }
    const bool fall = was && !side_present_[i], rise = !was && side_present_[i];
    if (!fall && !(rise && cfg::EDGE_RISE)) continue;
    const float axle_off = fall ? cfg::WALL_END_AXLE : cfg::WALL_START_AXLE;
    int* last = fall ? last_edge_k_ : last_rise_k_;
    const int k = static_cast<int>(std::lround((ms_ + axle_off - mv_.edge_phase) / cfg::CELL));
    if (k == last[0] || k == last[1]) continue;
    const float expect = mv_.edge_phase + k * cfg::CELL - axle_off;
    const float corr = expect - ms_;
    if (std::fabs(corr) > cfg::EDGE_CORR_MAX) continue;
    ms_ += corr;
    s_ref_ += corr;
    s_meas_ += corr;
    last[i] = k;
  }
}

void Motion::diag_track() {
  const int ids[2] = {cfg::IR_L, cfg::IR_R};
  if (lat_valid_) lat_est_ -= v_ref_ * std::sin(th_meas_ - th_ref_) * DT;
  for (int i = 0; i < 2; i++) {
    const float d = ir_f_[ids[i]];
    if (!darmed_[i]) {
      if (d > cfg::DIAG_POST_SEEN) { darmed_[i] = true; dmin_[i] = cfg::IR_NONE; }
      continue;
    }
    if (d < dmin_[i]) { dmin_[i] = d; ds_[i] = s_meas_; }
    if (dmin_[i] < cfg::DIAG_POST_SEEN && d > dmin_[i] + 8.0f) {
      dval_[i] = dmin_[i];
      dvalid_[i] = true;
      darmed_[i] = false;
      const int o = 1 - i;
      const bool other = dvalid_[o] && std::fabs(ds_[o] - ds_[i]) < 1.5f * cfg::DIAG_STEP;
      float e;
      if (other) e = 0.5f * ((i == 0 ? dval_[0] : dval_[0]) - dval_[1]);
      else e = i == 0 ? dval_[0] - cfg::DIAG_POST_NOM : cfg::DIAG_POST_NOM - dval_[1];
      lat_est_ = e;
      lat_valid_ = true;
#ifdef MM_DEBUG_LOG
      dbg_e_ = e; dbg_side_ = i; dbg_new_ = true;
#endif
    }
  }
  diag_active_ = lat_valid_ && v_ref_ > 100.0f;
}

void Motion::step_move() {
  t_move_ += DT;
  switch (mv_.type) {
    case MoveType::STRAIGHT:
    case MoveType::DIAG: {
      if ((mv_.flags & F_FGUARD) && !guard_hit_ && front_guard()) break;
      const float rem = mv_.dist - ms_;
      float vt = std::fmin(mv_.v_max, std::sqrt(std::fmax(0.0f, mv_.v_end * mv_.v_end + 2.0f * mv_.decel * rem)));
      if (mv_.v_end < 1.0f && rem > 0.0f) vt = std::fmax(vt, V_CREEP);
      const float a_avail = (cfg::V_HEADROOM * vbat_ - cfg::KS_MOTOR - cfg::KV_MOTOR * v_ref_) / cfg::KA_MOTOR;
      const float acc = clampf(a_avail, 0.0f, mv_.accel);
      const float vn = v_ref_ < vt ? std::fmin(v_ref_ + acc * DT, vt) : vt;
      a_ref_ = (vn - v_ref_) / DT;
      v_ref_ = vn;
      w_ref_ = al_ref_ = 0;
      ms_ += v_ref_ * DT;
      s_ref_ += v_ref_ * DT;
      wall_active_ = (mv_.flags & F_WALL) && mv_.type == MoveType::STRAIGHT && v_ref_ > 50.0f;
      if (mv_.type == MoveType::DIAG && cfg::DIAG_POST_CTRL) diag_track();
      if (mv_.flags & F_EDGE) edge_correction();
      presample();
      if ((mv_.flags & F_SENSE) && !sensed_ && ms_ >= mv_.sense_at) sense();
      if (ms_ >= mv_.dist) {
        if (mv_.v_end < 1.0f) {
          s_ref_ -= ms_ - mv_.dist;
          v_ref_ = a_ref_ = 0;
          ms_ = 0;
        } else {
          ms_ -= mv_.dist;
        }
        wall_active_ = false;
        active_ = false;
        moves_done_++;
      }
      break;
    }
    case MoveType::TURN: {
      const TurnGeom& g = g_turn[mv_.turn];
      const float v = mv_.v_max;
      a_ref_ = 0;
      v_ref_ = v;
      ms_ += v * DT;
      s_ref_ += v * DT;
      const float w = mv_.dir * turn_curvature(g, ms_) * v;
      al_ref_ = (w - w_ref_) / DT;
      w_ref_ = w;
      th_ref_ += w * DT;
      wall_active_ = false;
      presample();
      if ((mv_.flags & F_SENSE) && !sensed_ && ms_ >= mv_.sense_at) sense();
      if (ms_ >= mv_.dist) {
        th_ref_ = th_start_ + mv_.dir * g.theta;
        w_ref_ = al_ref_ = 0;
        ms_ -= mv_.dist;
        active_ = false;
        moves_done_++;
      }
      break;
    }
    case MoveType::PIVOT: {
      v_ref_ = a_ref_ = 0;
      wall_active_ = false;
      const float A = std::fabs(mv_.dist), sg = sgnf(mv_.dist);
      if (!settling_) {
        const float rem = A - phi_;
        const float wt = std::fmin(cfg::PIVOT_W_MAX, std::sqrt(std::fmax(0.0f, 2.0f * cfg::PIVOT_ALPHA * rem)));
        float wn = w_piv_ < wt ? std::fmin(w_piv_ + cfg::PIVOT_ALPHA * DT, wt) : wt;
        if (rem > 1e-3f) wn = std::fmax(wn, 0.5f);
        al_ref_ = sg * (wn - w_piv_) / DT;
        w_piv_ = wn;
        phi_ += wn * DT;
        w_ref_ = sg * wn;
        th_ref_ = th_start_ + sg * std::fmin(phi_, A);
        if (phi_ >= A) {
          settling_ = true;
          t_move_ = 0;
          w_ref_ = al_ref_ = 0;
          th_ref_ = th_start_ + mv_.dist;
        }
      } else if ((std::fabs(th_ref_ - th_meas_) < 0.03f && std::fabs(w_meas_) < 0.5f) || t_move_ > 0.3f) {
        s_ref_ = s_meas_;
        active_ = false;
        moves_done_++;
      }
      break;
    }
    case MoveType::FRONT_ALIGN:
      v_ref_ = a_ref_ = w_ref_ = al_ref_ = 0;
      wall_active_ = false;
      if (t_move_ >= mv_.dist || (t_move_ > (cfg::FA_FILTER ? 0.2f : 0.08f) && std::fabs(fa_ang_) < 0.004f && std::fabs(fa_dist_) < 1.0f &&
                                  std::fabs(w_meas_) < 0.2f && std::fabs(v_meas_) < 15.0f)) {
        front_align_ = false;
        if (std::fabs(fa_ang_) < 0.012f) th_meas_ = th_ref_ + fa_ang_;
        s_ref_ = s_meas_;
        active_ = false;
        moves_done_++;
      }
      break;
    case MoveType::WAIT:
      v_ref_ = a_ref_ = w_ref_ = al_ref_ = 0;
      wall_active_ = false;
      if (t_move_ >= mv_.dist) { active_ = false; moves_done_++; }
      break;
  }
}

void Motion::heading_from_wall(float e) {
  constexpr float STEP = 5.0f;
  constexpr int WIN = 8;
  if (wh_n_ > 0 && s_meas_ - wh_s_[(wh_n_ - 1) % WH_N] < STEP) return;
  wh_s_[wh_n_ % WH_N] = s_meas_;
  wh_e_[wh_n_ % WH_N] = e;
  wh_n_++;
  if (wh_n_ <= WIN) return;
  const int a = (wh_n_ - 1 - WIN) % WH_N, b = (wh_n_ - 1) % WH_N;
  const float ds = wh_s_[b] - wh_s_[a];
  if (ds < 0.8f * WIN * STEP) return;
  const float slope = (wh_e_[b] - wh_e_[a]) / ds;
  const float off = (th_meas_ - th_ref_) + slope;
  if (std::fabs(off) < 0.1f) th_meas_ -= cfg::WALL_HEADING_GAIN * off * STEP / std::fmax(v_ref_, 50.0f);
}

void Motion::control(const TickIn& in, TickOut& out) {
  const float e_s = s_ref_ - s_meas_;
  const float e_v = v_ref_ - v_meas_;
  i_pos_ = clampf(i_pos_ + cfg::KI_POS * e_s * DT, -cfg::I_LIMIT_V, cfg::I_LIMIT_V);
  float uf = cfg::KV_MOTOR * v_ref_ + cfg::KA_MOTOR * a_ref_ + cfg::KP_POS * e_s + cfg::KD_POS * e_v + i_pos_;

  float wall = 0;
  if (wall_active_) {
    bool valid = false;
    const float e = wall_error(valid);
    if (valid) {
      const float kw = std::fmin(cfg::KWALL_MAX, cfg::WALL_LAT_BW / std::fmax(v_ref_, 150.0f));
      wall = clampf(kw * e, -cfg::WALL_ANG_LIMIT, cfg::WALL_ANG_LIMIT);
      heading_from_wall(e);
    } else {
      wh_n_ = 0;
    }
  } else {
    wh_n_ = 0;
  }
  if (diag_active_ && mv_.type == MoveType::DIAG && active_)
    wall = clampf(cfg::KWALL_DIAG * lat_est_, -cfg::WALL_ANG_LIMIT, cfg::WALL_ANG_LIMIT);
  wall_dbg_ = wall;
  const float half = 0.5f * cfg::TRACK_MM;
  const float e_th = th_ref_ + wall - th_meas_;
  const float e_w = w_ref_ - w_meas_;
  i_ang_ = clampf(i_ang_ + cfg::KI_ANG * e_th * DT, -cfg::I_LIMIT_V, cfg::I_LIMIT_V);
  float ur = cfg::KV_MOTOR * w_ref_ * half + cfg::KA_MOTOR * cfg::ROT_INERTIA * al_ref_ * half +
             cfg::KP_ANG * e_th + cfg::KD_ANG * e_w + i_ang_;

  if (front_align_) {
    i_pos_ = i_ang_ = 0;
    const cfg::IrPose& pl = cfg::IR_POSE[cfg::IR_DL];
    const cfg::IrPose& pr = cfg::IR_POSE[cfg::IR_DR];
    const float rl = ir_f_[cfg::IR_DL], rr = ir_f_[cfg::IR_DR];
    const float xl = pl.fwd + rl * std::cos(pl.ang), yl = pl.lat + rl * std::sin(pl.ang);
    const float xr = pr.fwd + rr * std::cos(pr.ang), yr = pr.lat + rr * std::sin(pr.ang);
    const float a_raw = std::atan2(xl - xr, yl - yr);
    const float d_raw = 0.5f * (xl + xr) - cfg::FRONT_ALIGN_X;
    if (t_move_ <= 2 * DT || !cfg::FA_FILTER) { fa_ang_ = a_raw; fa_dist_ = d_raw; }
    fa_ang_ += 0.02f * (a_raw - fa_ang_);
    fa_dist_ += 0.05f * (d_raw - fa_dist_);
#ifdef MM_DEBUG_LOG
    dbg_e_ = a_raw;
#endif
    uf = clampf(cfg::KP_FRONT_DIST * fa_dist_ - cfg::KD_POS * v_meas_, -1.5f, 1.5f);
    ur = clampf(-cfg::KP_FRONT_ANG * fa_ang_, -1.5f, 1.5f) - cfg::KD_ANG * w_meas_;
    if (std::fabs(fa_dist_) > 0.7f) uf += cfg::KS_MOTOR * sgnf(uf);
    if (std::fabs(fa_ang_) > 0.004f) ur += cfg::KS_MOTOR * sgnf(ur);
  }

  float vl = uf - ur, vr = uf + ur;
  const float wl = v_ref_ - w_ref_ * half, wr = v_ref_ + w_ref_ * half;
  if (std::fabs(wl) > 5.0f) vl += cfg::KS_MOTOR * sgnf(wl);
  if (std::fabs(wr) > 5.0f) vr += cfg::KS_MOTOR * sgnf(wr);
  if (!front_align_ && v_ref_ == 0.0f && w_ref_ == 0.0f) {
    if (std::fabs(e_th) > 0.006f) {
      const float k = 0.8f * cfg::KS_MOTOR * sgnf(e_th);
      vl -= k;
      vr += k;
    }
    if (std::fabs(e_s) > 0.3f) {
      const float k = 0.8f * cfg::KS_MOTOR * sgnf(e_s);
      vl += k;
      vr += k;
    }
  }
  const float vb = in.vbat > 3.0f ? in.vbat : cfg::VBAT_NOMINAL;
  out.duty_l = cfg::MOTOR_SIGN_L * clampf(vl / vb, -cfg::DUTY_LIMIT, cfg::DUTY_LIMIT);
  out.duty_r = cfg::MOTOR_SIGN_R * clampf(vr / vb, -cfg::DUTY_LIMIT, cfg::DUTY_LIMIT);

  if (std::fabs(e_s) > max_es_) max_es_ = std::fabs(e_s);
  const bool bad = !front_align_ &&
                   (std::fabs(e_s) > cfg::FAULT_POS_MM || std::fabs(th_ref_ - th_meas_) > cfg::FAULT_ANG_RAD);
  fault_cnt_ = bad ? static_cast<uint16_t>(fault_cnt_ + 1) : 0;
  if (fault_cnt_ > cfg::FAULT_MS) {
    fault_ = true;
    active_ = false;
    tail_ = head_;
  }
}

void Motion::tick(const TickIn& in, TickOut& out) {
  out.duty_l = out.duty_r = 0;
  vbat_ = in.vbat > 3.0f ? in.vbat : cfg::VBAT_NOMINAL;
  if (!enc_init_) { enc_l0_ = in.enc_l; enc_r0_ = in.enc_r; enc_init_ = true; }
  const int32_t dcl = in.enc_l - enc_l0_, dcr = in.enc_r - enc_r0_;
  enc_l0_ = in.enc_l;
  enc_r0_ = in.enc_r;
  const float dl = cfg::ENC_SIGN_L * dcl / cfg::COUNTS_PER_MM_L;
  const float dr = cfg::ENC_SIGN_R * dcr / cfg::COUNTS_PER_MM_R;
  const float ds = 0.5f * (dl + dr);
  s_meas_ += ds;
  v_meas_ = 0.6f * v_meas_ + 0.4f * (ds / DT);

  for (int i = 0; i < cfg::IR_COUNT; i++) {
    const float r = std::fmin(in.ir[i], 300.0f);
    if (r != ir_raw_[i]) {
      if (i == cfg::IR_L || i == cfg::IR_R) {
        if (std::fabs(r - ir_raw_[i]) > cfg::SIDE_JUMP_MM) blank_until_[i - cfg::IR_L] = s_meas_ + 30.0f;
      }
      ir_prev_[i] = ir_raw_[i];
      ir_raw_[i] = r;
    }
    ir_f_[i] = 0.6f * ir_f_[i] + 0.4f * r;
  }
  {
    const float a_sf = std::fmin(1.0f, std::fabs(ds) / cfg::SIDE_FILT_MM + 0.002f);
    const int sid[2] = {cfg::IR_L, cfg::IR_R};
    for (int k = 0; k < 2; k++) {
      const float r = ir_raw_[sid[k]];
      if (r >= 299.0f || ir_sf_[k] >= 299.0f) ir_sf_[k] = r;
      else ir_sf_[k] += a_sf * (r - ir_sf_[k]);
    }
  }

  if (cal_left_ > 0) {
    cal_sum_ += in.gyro_z;
    cal_n_++;
    if (--cal_left_ == 0) gyro_bias_ = static_cast<float>(cal_sum_ / cal_n_);
    w_meas_ = 0;
    return;
  }
  const float w_raw = (in.gyro_z - (in.gyro_enc ? 0.0f : gyro_bias_)) * cfg::GYRO_SIGN * cfg::GYRO_SCALE;
  th_meas_ += w_raw * DT;
  w_meas_ = in.gyro_enc ? 0.6f * w_meas_ + 0.4f * w_raw : w_raw;

  if (!enabled_ || fault_) return;

  if (!active_) start_next();
  if (active_) {
    step_move();
    if (!active_ && head_ != tail_) { start_next(); }
  } else if (v_ref_ > 0.0f) {
    const float vn = std::fmax(0.0f, v_ref_ - UNDERRUN_DECEL * DT);
    a_ref_ = (vn - v_ref_) / DT;
    v_ref_ = vn;
    ms_ += v_ref_ * DT;
    s_ref_ += v_ref_ * DT;
    w_ref_ = al_ref_ = 0;
    wall_active_ = false;
  } else {
    a_ref_ = w_ref_ = al_ref_ = 0;
    wall_active_ = false;
  }
  control(in, out);
}

}
